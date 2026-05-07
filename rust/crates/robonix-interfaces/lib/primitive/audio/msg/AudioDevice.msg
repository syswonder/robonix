# A single audio device an implementation can drive (input or output).
# Each impl exposes its own id space — sounddevice uses integer indices,
# ALSA uses "hw:0,0", PulseAudio uses sink names, etc. The TUI just
# round-trips the id string back via SelectAudioDevice; only the impl
# needs to know what the id means.

string id                 # impl-specific stable identifier
string name               # human-readable name (shown in pickers)
string kind               # "input" | "output" | "duplex"
bool is_default           # true if this is the OS-level default for its kind
uint32 channels           # max channels at the impl's preferred rate
string note               # free-form hint: "bluetooth" / "usb" / "airpods"
                          # — pickers can warn or auto-skip on these
