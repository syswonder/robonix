# One OS-level audio device that a multi-device audio cap's driver can
# route to internally. NOT a robonix cap — robonix caps stay 1:1 with
# packages. This message describes the driver's internal routing table:
# `com.robonix.primitive.audio.alsa` (one cap) drives many ALSA "hw:X,Y"
# devices, this is one row of that table.
#
# Each impl owns its own id space — sounddevice uses integer indices,
# ALSA uses "hw:0,0", PulseAudio uses sink names, etc. The id string
# round-trips back via SelectAudioDevice; only the impl needs to know
# what it means.

string id                 # impl-specific stable identifier
string name               # human-readable name (shown in pickers)
string kind               # "input" | "output" | "duplex"
bool is_default           # true if this is the OS-level default for its kind
uint32 channels           # max channels at the impl's preferred rate
string note               # free-form hint: "bluetooth" / "usb" / "airpods"
                          # — pickers can warn or auto-skip on these
