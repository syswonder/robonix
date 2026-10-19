# report_available=false is valid before the first successful reference sample;
# consumers must ignore the reference, observed_steady_ns, and domain arrays.
# Encoders set those ignored payloads to their default or empty values.
bool report_available
uint64 observed_steady_ns
chronos/msg/ReferenceQuality reference
chronos/msg/RefreshHealth refresh_health
chronos/msg/DirectDomainQuality[] direct_domains
chronos/msg/AffineDomainQuality[] affine_domains
