import base64
import binascii
import os
import unittest
from unittest.mock import Mock, patch

from speech_service.tencent_cloud import (
    TencentRealtimeASRBackend,
    TencentTTSBackend,
    _TTS_MAX_SEGMENTS,
    _hard_split_tts_text,
    _split_tts_text,
)
from speech_service.tts_errors import TTSInputError
from speech_service.tts_runtime import (
    bind_tts_request_active,
    reset_tts_request_active,
)

_BASE_ENV = {
    "TENCENTCLOUD_SECRET_ID": "test-id",
    "TENCENTCLOUD_SECRET_KEY": "test-key",
}

_HAN_A = "\N{CJK UNIFIED IDEOGRAPH-7532}"
_HAN_B = "\N{CJK UNIFIED IDEOGRAPH-4E59}"
_CJK_FULL_STOP = "\N{IDEOGRAPHIC FULL STOP}"
_CJK_COMMA = "\N{FULLWIDTH COMMA}"
_CJK_EXCLAMATION = "\N{FULLWIDTH EXCLAMATION MARK}"
_MIXED_TEXT = (
    f"{_HAN_A}{_HAN_B}{_CJK_COMMA}Robot status. "
    f"{_HAN_B}{_HAN_A}{_CJK_EXCLAMATION}Task execution continues."
)
_STREAM_TEXT = (
    f"{_HAN_A}{_HAN_B}{_CJK_FULL_STOP}"
    f"{_HAN_B}{_HAN_A}{_CJK_COMMA}"
    f"{_HAN_A}{_HAN_B}{_CJK_FULL_STOP}"
    f"{_HAN_B}{_HAN_A}{_CJK_FULL_STOP}"
)
_SHORT_CJK_TEXT = f"{_HAN_A}{_HAN_B}{_HAN_A}{_HAN_B}"
_SENTENCE_CASE = (
    f"{_HAN_A}{_HAN_B}{_CJK_FULL_STOP}"
    f"{_HAN_A}{_HAN_B}{_HAN_A}{_HAN_B}{_CJK_COMMA}{_HAN_A}{_HAN_B}"
)
_CLAUSE_CASE = (
    f"{_HAN_A}{_HAN_B}{_HAN_A}{_HAN_B}{_CJK_COMMA}"
    f"{_HAN_B}{_HAN_A}{_HAN_B}{_HAN_A}{_HAN_B}{_HAN_A}{_HAN_B}"
)


def _fake_pcm(text: str) -> bytes:
    raw = text.encode("utf-8")
    return raw if len(raw) % 2 == 0 else raw + b"\x00"


class TencentStreamingTranscriptTest(unittest.TestCase):
    def test_revisable_hypotheses_emit_only_final_snapshot(self):
        last = ""
        emitted = []
        for text, final in [
            ("okay, I may", False),
            ("okay, I have seen him", False),
            ("okay, I have seen it start", True),
        ]:
            event, last = TencentRealtimeASRBackend._to_incremental_event(
                {"text": text, "is_final": final, "event_type": int(final)}, last
            )
            if event is not None:
                emitted.append(event["text"])

        self.assertEqual(emitted, ["okay, I have seen it start"])

    def test_nonfinal_duplicate_snapshot_is_suppressed(self):
        event, last = TencentRealtimeASRBackend._to_incremental_event(
            {"text": "still there", "is_final": False}, "still there"
        )
        self.assertIsNone(event)
        self.assertEqual(last, "still there")


class TencentTTSChunkingTest(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        """Create a Tencent backend with fake credentials and a small limit."""
        self._env = patch.dict(
            os.environ,
            {
                "TENCENTCLOUD_SECRET_ID": "test-id",
                "TENCENTCLOUD_SECRET_KEY": "test-key",
                "TENCENT_TTS_MAX_CHARS": "10",
            },
            clear=True,
        )
        self._env.start()
        self.addCleanup(self._env.stop)
        self.backend = TencentTTSBackend()

    def _install_echo_response(self):
        """Return request text as audio bytes and record each provider payload."""
        payloads = []

        def echo(payload):
            if len(payload["Text"]) > self.backend.max_text_chars:
                raise RuntimeError("UnsupportedOperation.TextTooLong")
            payloads.append(payload)
            audio = base64.b64encode(_fake_pcm(payload["Text"])).decode("ascii")
            return {"Audio": audio}

        self.backend._post_json = echo
        return payloads

    async def test_text_around_boundary_uses_expected_request_count(self):
        """N-1/N/N+1 inputs use one, one, and two provider requests."""
        for length, request_count in ((9, 1), (10, 1), (11, 2)):
            with self.subTest(length=length):
                text = _HAN_A * length
                payloads = self._install_echo_response()
                audio = await self.backend.synthesize(text)
                sent = [payload["Text"] for payload in payloads]
                self.assertEqual(len(sent), request_count)
                self.assertEqual("".join(sent), text)
                self.assertEqual(audio, b"".join(_fake_pcm(part) for part in sent))

    async def test_long_chinese_and_mixed_text_are_complete_and_bounded(self):
        """Long Chinese and mixed input reaches the provider in bounded order."""
        for text in (
            _HAN_A * 244,
            _MIXED_TEXT,
        ):
            with self.subTest(text=text):
                payloads = self._install_echo_response()
                audio = await self.backend.synthesize(text)
                sent = [payload["Text"] for payload in payloads]
                self.assertEqual("".join(sent), text)
                self.assertTrue(all(0 < len(segment) <= 10 for segment in sent))
                self.assertEqual(audio, b"".join(_fake_pcm(part) for part in sent))
                self.assertTrue(
                    all(payload["PrimaryLanguage"] == 1 for payload in payloads)
                )

    async def test_stream_yields_first_segment_before_requesting_the_next(self):
        """Streaming emits a segment before starting the following request."""
        text = _STREAM_TEXT
        expected = _split_tts_text(text, 10)
        payloads = self._install_echo_response()
        stream = self.backend.synthesize_stream(text)

        first = await anext(stream)
        self.assertEqual(first, _fake_pcm(expected[0]))
        self.assertEqual([payload["Text"] for payload in payloads], expected[:1])

        remaining = [chunk async for chunk in stream]
        self.assertEqual(
            b"".join([first, *remaining]),
            b"".join(_fake_pcm(part) for part in expected),
        )
        self.assertEqual([payload["Text"] for payload in payloads], expected)

    async def test_short_stream_uses_one_request_and_yields_complete_audio(self):
        """Tencent streaming keeps short input in one complete request."""
        text = _SHORT_CJK_TEXT
        payloads = self._install_echo_response()

        audio = b"".join(
            [chunk async for chunk in self.backend.synthesize_stream(text)]
        )

        self.assertEqual([payload["Text"] for payload in payloads], [text])
        self.assertEqual(audio, _fake_pcm(text))

    async def test_stream_drains_large_segment_before_requesting_next(self):
        """All 4096-byte chunks for one segment precede the next request."""
        payloads = []

        def large_response(payload):
            payloads.append(payload)
            return {"Audio": base64.b64encode(b"\x00\x00" * 4097).decode("ascii")}

        self.backend._post_json = large_response
        stream = self.backend.synthesize_stream(_HAN_A * 11)
        first_segment = [await anext(stream) for _ in range(3)]

        self.assertEqual([len(chunk) for chunk in first_segment], [4096, 4096, 2])
        self.assertEqual([payload["Text"] for payload in payloads], [_HAN_A * 10])

        self.assertEqual(len(await anext(stream)), 4096)
        self.assertEqual(
            [payload["Text"] for payload in payloads],
            [_HAN_A * 10, _HAN_A],
        )
        await stream.aclose()

    async def test_failure_identifies_segment_and_keeps_original_exception(self):
        """A failed provider request names its segment and retains its cause."""
        provider_error = ValueError("provider failed")
        calls = []

        def fail_second(payload):
            calls.append(payload)
            if len(calls) == 2:
                raise provider_error
            return {"Audio": base64.b64encode(b"\x00\x00").decode("ascii")}

        self.backend._post_json = fail_second
        with self.assertRaisesRegex(RuntimeError, r"segment 2/3 failed") as raised:
            await self.backend.synthesize(_HAN_A * 21)
        self.assertIs(raised.exception.__cause__, provider_error)

    async def test_stream_failure_identifies_later_segment(self):
        """Streaming preserves context after yielding an earlier segment."""
        provider_error = RuntimeError("stream provider failed")
        calls = []

        def fail_second(payload):
            calls.append(payload)
            if len(calls) == 2:
                raise provider_error
            return {"Audio": base64.b64encode(b"\x00\x00").decode("ascii")}

        self.backend._post_json = fail_second
        stream = self.backend.synthesize_stream(_HAN_A * 21)
        self.assertEqual(await anext(stream), b"\x00\x00")
        with self.assertRaisesRegex(RuntimeError, r"segment 2/3 failed") as raised:
            await anext(stream)
        self.assertIs(raised.exception.__cause__, provider_error)

    async def test_rejects_malformed_or_invalid_pcm_audio(self):
        """Malformed Base64 and odd pcm_s16le payloads fail explicitly."""
        cases = (
            ("%%%%", "base64", binascii.Error),
            ("YQ==", "odd-length", RuntimeError),
        )
        for encoded, detail, cause_type in cases:
            with self.subTest(encoded=encoded):
                self.backend._post_json = lambda _payload, audio=encoded: {
                    "Audio": audio
                }
                with self.assertRaisesRegex(
                    RuntimeError, r"segment 1/1 failed"
                ) as raised:
                    await self.backend.synthesize("short")
                self.assertIsInstance(raised.exception.__cause__, cause_type)
                self.assertIn(detail, str(raised.exception.__cause__).lower())

    async def test_invalid_or_unsafe_text_is_rejected_before_provider_calls(self):
        """Never drop or send contentless text merely to satisfy a tiny limit."""
        self.backend.max_text_chars = 1
        payloads = self._install_echo_response()

        cases = (
            (_HAN_A + _CJK_FULL_STOP, "punctuation- or whitespace-only"),
            (_CJK_FULL_STOP * 2, "speakable content"),
            ("", "must not be empty"),
        )
        for text, detail in cases:
            with self.subTest(text=text), self.assertRaisesRegex(ValueError, detail):
                await self.backend.synthesize(text)

        self.assertEqual(payloads, [])

    async def test_total_text_and_request_limits_prevent_provider_fanout(self):
        """Bound request cost before starting any partial synthesis."""
        payloads = self._install_echo_response()
        self.backend.max_total_text_chars = 20

        with self.assertRaisesRegex(TTSInputError, "exceeds.*20"):
            await self.backend.synthesize(_HAN_A * 21)

        self.backend.max_total_text_chars = 5_000
        self.backend.max_text_chars = 1
        with self.assertRaisesRegex(TTSInputError, "requires 41 requests"):
            await self.backend.synthesize(_HAN_A * 41)
        self.assertEqual(payloads, [])

    async def test_dense_sentence_punctuation_stays_within_request_budget(self):
        """Valid long text must not be rejected for inefficient natural splits."""
        # Reviewer fixture: naive sentence cuts previously alternated 140/2 and
        # produced 70 segments, tripping _TTS_MAX_SEGMENTS even though length
        # is under 5000 and a hard budget split needs only 36 requests.
        text = ("a" * 141 + ".") * 35
        self.assertLessEqual(len(text), 5_000)
        natural = _split_tts_text(text, 140)
        hard = _hard_split_tts_text(text, 140)
        self.assertLessEqual(len(natural), _TTS_MAX_SEGMENTS)
        self.assertLessEqual(len(hard), _TTS_MAX_SEGMENTS)
        self.assertEqual("".join(natural), text)
        self.assertEqual("".join(hard), text)

        with patch.dict(
            os.environ,
            {
                "TENCENTCLOUD_SECRET_ID": "test-id",
                "TENCENTCLOUD_SECRET_KEY": "test-key",
                "TENCENT_TTS_MAX_CHARS": "140",
                "TENCENT_TTS_MAX_TOTAL_CHARS": "5000",
            },
            clear=True,
        ):
            backend = TencentTTSBackend()
        payloads = []

        def echo(payload):
            payloads.append(payload)
            return {
                "Audio": base64.b64encode(_fake_pcm(payload["Text"])).decode("ascii")
            }

        backend._post_json = echo
        audio = await backend.synthesize(text)
        sent = [payload["Text"] for payload in payloads]
        self.assertEqual("".join(sent), text)
        self.assertLessEqual(len(sent), _TTS_MAX_SEGMENTS)
        self.assertEqual(audio, b"".join(_fake_pcm(part) for part in sent))

    async def test_request_budget_falls_back_to_hard_split(self):
        """If natural splits exceed the cap, reuse a bounded hard split."""
        text = _HAN_A * 50
        with patch.dict(os.environ, _BASE_ENV, clear=True):
            backend = TencentTTSBackend()
        backend.max_text_chars = 10
        bloated = [text[i : i + 1] for i in range(len(text))]
        self.assertGreater(len(bloated), _TTS_MAX_SEGMENTS)
        with patch(
            "speech_service.tencent_cloud._split_tts_text", return_value=bloated
        ):
            segments = backend._segments_for_provider(text)
        self.assertEqual(segments, _hard_split_tts_text(text, 10))
        self.assertLessEqual(len(segments), _TTS_MAX_SEGMENTS)
        self.assertEqual("".join(segments), text)

    async def test_cancellation_stops_before_later_stream_segments(self):
        """Streaming honors the same liveness gate as one-shot synthesis."""
        payloads = self._install_echo_response()
        active = True

        def is_active():
            return active

        token = bind_tts_request_active(is_active)
        try:
            stream = self.backend.synthesize_stream(_HAN_A * 21)
            self.assertEqual(await anext(stream), _fake_pcm(_HAN_A * 10))
            active = False
            with self.assertRaisesRegex(RuntimeError, r"cancelled before segment 2/3"):
                await anext(stream)
        finally:
            reset_tts_request_active(token)
        self.assertEqual([payload["Text"] for payload in payloads], [_HAN_A * 10])

    async def test_pcm_limits_bound_one_shot_and_segment_memory(self):
        """Reject unexpectedly large decoded audio before it grows unbounded."""
        payloads = self._install_echo_response()
        with (
            patch("speech_service.tencent_cloud._TTS_MAX_TOTAL_PCM_BYTES", 32),
            self.assertRaisesRegex(RuntimeError, "one-shot safety limit"),
        ):
            await self.backend.synthesize(_HAN_A * 11)
        self.assertEqual(len(payloads), 2)

        payloads.clear()
        with (
            patch("speech_service.tencent_cloud._TTS_MAX_SEGMENT_PCM_BYTES", 2),
            patch("speech_service.tencent_cloud.base64.b64decode") as decode,
        ):
            stream = self.backend.synthesize_stream(_HAN_A)
            with self.assertRaisesRegex(RuntimeError, "encoded audio exceeds"):
                await anext(stream)
        decode.assert_not_called()
        self.assertEqual(len(payloads), 1)


class TencentTTSTextSplitterTest(unittest.TestCase):
    def test_boundary_and_hard_split(self):
        self.assertEqual(_split_tts_text(_HAN_A * 10, 10), [_HAN_A * 10])
        self.assertEqual(_split_tts_text(_HAN_A * 11, 10), [_HAN_A * 10, _HAN_A])

    def test_default_safe_limit_splits_reported_chinese_response(self):
        text = _HAN_A * 244
        segments = _split_tts_text(text, 140)
        self.assertEqual([len(segment) for segment in segments], [140, 104])
        self.assertEqual("".join(segments), text)

    def test_sentence_break_precedes_later_comma(self):
        """Sentence punctuation wins over a later comma in the same window."""
        text = _SENTENCE_CASE
        segments = _split_tts_text(text, 8)
        self.assertEqual(segments[0], _HAN_A + _HAN_B + _CJK_FULL_STOP)
        self.assertEqual("".join(segments), text)
        self.assertTrue(all(len(segment) <= 8 for segment in segments))

    def test_english_punctuation_newline_comma_and_word_boundaries(self):
        """Natural mixed-language boundaries are preferred before hard cuts."""
        cases = (
            ("Hello!World?", 7, ["Hello!", "World?"]),
            ("abc\ndefghijk", 6, ["abc\n", "defghi", "jk"]),
            (
                _CLAUSE_CASE,
                6,
                [
                    _HAN_A + _HAN_B + _HAN_A + _HAN_B + _CJK_COMMA,
                    _HAN_B + _HAN_A + _HAN_B + _HAN_A + _HAN_B + _HAN_A,
                    _HAN_B,
                ],
            ),
            ("Robot status.", 10, ["Robot ", "status."]),
        )
        for text, limit, expected in cases:
            with self.subTest(text=text):
                self.assertEqual(_split_tts_text(text, limit), expected)

    def test_trailing_punctuation_is_not_isolated_after_hard_cut(self):
        text = _HAN_A * 10 + _CJK_FULL_STOP
        segments = _split_tts_text(text, 10)
        self.assertEqual(segments, [_HAN_A * 9, _HAN_A + _CJK_FULL_STOP])
        self.assertTrue(all(any(char.isalnum() for char in part) for part in segments))

    def test_rejects_non_positive_limit(self):
        with self.assertRaisesRegex(ValueError, "greater than zero"):
            _split_tts_text("text", 0)

    def test_ignores_tiny_follow_up_sentence_fragments_on_long_remainder(self):
        """Avoid 140/2 alternation when a later hard cut stays within budget."""
        text = ("a" * 141 + ".") * 35
        segments = _split_tts_text(text, 140)
        self.assertEqual("".join(segments), text)
        self.assertLessEqual(len(segments), _TTS_MAX_SEGMENTS)
        self.assertTrue(all(len(segment) <= 140 for segment in segments))
        # The previous bug emitted a two-character leftover after nearly every
        # hard cut (~70 segments). Keep that explosion from returning.
        tiny = [segment for segment in segments if len(segment) <= 2]
        self.assertLessEqual(len(tiny), 1)


class TencentTTSConfigTest(unittest.TestCase):
    def test_defaults_to_provider_safe_limit_and_pcm(self):
        with patch.dict(os.environ, _BASE_ENV, clear=True):
            backend = TencentTTSBackend()
        self.assertEqual(backend.max_text_chars, 140)
        self.assertEqual(backend.max_total_text_chars, 5_000)
        self.assertEqual(backend.codec, "pcm")
        self.assertIsNone(backend.primary_language)

    def test_rejects_limit_outside_provider_safe_range(self):
        """Invalid or unsafe limits fail during backend initialization."""
        cases = (
            ("0", "between 1 and 150"),
            ("151", "between 1 and 150"),
            ("x", "integer"),
        )
        for value, detail in cases:
            with (
                self.subTest(value=value),
                patch.dict(
                    os.environ,
                    {**_BASE_ENV, "TENCENT_TTS_MAX_CHARS": value},
                    clear=True,
                ),
                self.assertRaisesRegex(ValueError, detail),
            ):
                TencentTTSBackend()

    def test_rejects_invalid_total_text_limit(self):
        """The utterance-wide bound stays finite and configurable."""
        cases = (
            ("0", "between 1 and 6000"),
            ("6001", "between 1 and 6000"),
            ("x", "integer"),
        )
        for value, detail in cases:
            with (
                self.subTest(value=value),
                patch.dict(
                    os.environ,
                    {**_BASE_ENV, "TENCENT_TTS_MAX_TOTAL_CHARS": value},
                    clear=True,
                ),
                self.assertRaisesRegex(ValueError, detail),
            ):
                TencentTTSBackend()

    def test_rejects_invalid_primary_language_override(self):
        """Only Tencent's documented Chinese and English selectors are valid."""
        for value, detail in (("0", "1 or 2"), ("3", "1 or 2"), ("x", "integer")):
            with (
                self.subTest(value=value),
                patch.dict(
                    os.environ,
                    {**_BASE_ENV, "TENCENT_TTS_PRIMARY_LANGUAGE": value},
                    clear=True,
                ),
                self.assertRaisesRegex(ValueError, detail),
            ):
                TencentTTSBackend()

    def test_primary_language_auto_detection_and_valid_overrides(self):
        """Automatic and explicit language selection use one validated value."""
        cases = (
            ({}, "Robot status.", 2),
            ({}, _SHORT_CJK_TEXT, 1),
            ({"TENCENT_TTS_PRIMARY_LANGUAGE": "1"}, "Robot status.", 1),
            ({"TENCENT_TTS_PRIMARY_LANGUAGE": "2"}, _SHORT_CJK_TEXT, 2),
        )
        for extra_env, text, expected in cases:
            with (
                self.subTest(extra_env=extra_env, text=text),
                patch.dict(os.environ, {**_BASE_ENV, **extra_env}, clear=True),
            ):
                backend = TencentTTSBackend()
                self.assertEqual(backend._primary_language_for_text(text), expected)

    def test_rejects_non_pcm_codec(self):
        """Encoded Tencent output cannot satisfy the service's PCM contract."""
        with (
            patch.dict(
                os.environ,
                {**_BASE_ENV, "TENCENT_TTS_CODEC": "wav"},
                clear=True,
            ),
            self.assertRaisesRegex(ValueError, "requires.*pcm"),
        ):
            TencentTTSBackend()

    def test_rejects_non_16khz_sample_rate(self):
        """Speaker-bound raw PCM must use the service's fixed sample rate."""
        with (
            patch.dict(
                os.environ,
                {**_BASE_ENV, "TENCENT_TTS_SAMPLE_RATE": "8000"},
                clear=True,
            ),
            self.assertRaisesRegex(ValueError, "requires.*16000"),
        ):
            TencentTTSBackend()

    def test_provider_error_includes_request_id(self):
        """Tencent diagnostics retain the request identifier for operations."""
        provider_response = Mock()
        provider_response.raise_for_status.return_value = None
        provider_response.json.return_value = {
            "Response": {
                "Error": {"Code": "InvalidText", "Message": "bad text"},
                "RequestId": "request-211",
            }
        }
        with (
            patch.dict(os.environ, _BASE_ENV, clear=True),
            patch(
                "speech_service.tencent_cloud.requests.post",
                return_value=provider_response,
            ),
        ):
            backend = TencentTTSBackend()
            with self.assertRaisesRegex(RuntimeError, "request_id=request-211"):
                backend._post_json({"Text": "bad"})


if __name__ == "__main__":
    unittest.main()
