package radar

import "testing"

func validRaw() RawFrame {
	const nP, nS = 8, 64
	h := FrameHeader{
		SampleRateHz:    1e7,
		ChirpSlope:      1e12,
		PulseCount:      nP,
		SamplesPerPulse: nS,
		CarrierFreqHz:   77e9,
		PRISec:          1e-4,
	}
	pulses := make([]Pulse, nP)
	for i := range pulses {
		pulses[i] = Pulse{I: make([]float64, nS), Q: make([]float64, nS)}
	}
	return RawFrame{ID: 1, Header: h, Pulses: pulses}
}

func parseCfg() ParseConfig {
	return ParseConfig{
		MinPulses: 2, MaxPulses: 64,
		MinSamples: 16, MaxSamples: 1024,
		MinSampleRateHz: 1e6, MaxSampleRateHz: 1e9,
		MinChirpSlope: 1e9, MaxChirpSlope: 1e15,
		MinCarrierHz: 1e9, MaxCarrierHz: 100e9,
		MinPRISec: 1e-9, MaxPRISec: 1e-3,
	}
}

func TestParseAcceptsValid(t *testing.T) {
	if _, err := ParseFrame(validRaw(), parseCfg()); err != nil {
		t.Fatal(err)
	}
}

func TestParseRejectionKinds(t *testing.T) {
	cases := []struct {
		name   string
		mutate func(*RawFrame)
		kind   ParseErrorKind
	}{
		{"empty", func(r *RawFrame) { r.Pulses = nil }, ParseEmptyFrame},
		{"sample rate low", func(r *RawFrame) { r.Header.SampleRateHz = 1 }, ParseBadSampleRate},
		{"sample rate high", func(r *RawFrame) { r.Header.SampleRateHz = 2e9 }, ParseBadSampleRate},
		{"chirp slope zero", func(r *RawFrame) { r.Header.ChirpSlope = 0 }, ParseBadChirpSlope},
		{"pulse count low", func(r *RawFrame) { r.Header.PulseCount = 1 }, ParseBadPulseCount},
		{"pulse count high", func(r *RawFrame) { r.Header.PulseCount = 128 }, ParseBadPulseCount},
		{"samples zero", func(r *RawFrame) { r.Header.SamplesPerPulse = 0 }, ParseBadSamplesPerPulse},
		{"carrier zero", func(r *RawFrame) { r.Header.CarrierFreqHz = 0 }, ParseBadCarrierFreq},
		{"pri zero", func(r *RawFrame) { r.Header.PRISec = 0 }, ParseBadPRI},
		{"pulse mismatch", func(r *RawFrame) { r.Header.PulseCount = 9 }, ParsePulseCountMismatch},
		{"sample mismatch", func(r *RawFrame) {
			r.Pulses[0] = Pulse{I: make([]float64, 32), Q: make([]float64, 64)}
		}, ParseSampleCountMismatch},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			raw := validRaw()
			c.mutate(&raw)
			_, err := ParseFrame(raw, parseCfg())
			pe, ok := err.(*ParseError)
			if !ok {
				t.Fatalf("expected *ParseError, got %T %v", err, err)
			}
			if pe.Kind != c.kind {
				t.Fatalf("kind=%v want %v (%s)", pe.Kind, c.kind, pe.Msg)
			}
		})
	}
}

func TestParseExpectedDimensions(t *testing.T) {
	cfg := parseCfg()
	cfg.ExpectedPulses = 8
	cfg.ExpectedSamples = 64
	raw := validRaw()
	if _, err := ParseFrame(raw, cfg); err != nil {
		t.Fatal(err)
	}
	cfg.ExpectedPulses = 16
	raw = validRaw()
	_, err := ParseFrame(raw, cfg)
	if pe, ok := err.(*ParseError); !ok || pe.Kind != ParseParameterMismatch {
		t.Fatalf("got %v want ParameterMismatch", err)
	}
}

func TestIllegalWindowAndPadding(t *testing.T) {
	frame, err := ParseFrame(validRaw(), parseCfg())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := PreparePulses(frame, WindowConfig{RangeWindow: "bogus"}); err == nil {
		t.Fatal("expected error for unknown window")
	}
	if _, err := PreparePulses(frame, WindowConfig{RangePadTo: 8}); err == nil {
		t.Fatal("expected error when pad size < sample count")
	}
}
