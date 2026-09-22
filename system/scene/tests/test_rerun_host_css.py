# SPDX-License-Identifier: MulanPSL-2.0
"""The rerun host page must give the viewer a box to fill.

Two rules, each of which cost an afternoon, and neither of which fails
loudly -- the map draws, it is just the wrong size, so it reads as a
rendering bug and gets investigated as one.

1. `#host` is sized in percent, not by `position: absolute; inset: 0`. The
   viewer writes `position: relative` onto that element as it starts, which
   beats the stylesheet and leaves `inset` inert; the host then took its
   height from its content, and its content was a canvas sized to a default
   precisely because the host had no height to measure. 969x364 inside a
   969x902 frame.

2. The canvas is pinned with `!important`. `@rerun-io/web-viewer` writes
   `width: 640px; height: 360px` inline onto it, and inline beats a
   stylesheet without it. That default is what drew a 640x360 map into the
   corner of a 969x902 frame.

Asserting on CSS text is blunt, but the alternative is a browser, and the
failure is invisible to every check short of measuring the element.
"""
from scene_service import web


def _host_html() -> str:
    return web._RERUN_HOST_HTML


def test_host_is_sized_in_percent_not_by_inset():
    css = _host_html()
    assert "#host { width: 100%; height: 100%; }" in css, (
        "the host must have a definite size that survives the viewer "
        "setting `position: relative` on it")
    assert "#host { position: absolute" not in css, (
        "`position: absolute; inset: 0` is the regression: the viewer "
        "overrides `position` and the host collapses onto its content")


def test_the_canvas_is_pinned_over_the_viewers_inline_size():
    css = _host_html()
    body = css[css.index("#host > canvas"):css.index("#host > canvas") + 200]
    assert "width: 100% !important" in body, (
        "without !important the viewer's inline `width: 640px` wins")
    assert "height: 100% !important" in body, (
        "without !important the viewer's inline `height: 360px` wins")
