package frame

import (
	"bytes"
	"errors"
	"io"
	"testing"
)

func validFrame(seq int64) *RawFrame {
	h := Header{
		SampleRate:      10e6,
		ChirpSlope:      20e12,
		CarrierFreq:     77e9,
		PulseInterval:   40e-6,
		PulseCount:      4,
		SamplesPerPulse: 8,
	}
	f := &RawFrame{Seq: seq, Header: h}
	f.I = make([][]float64, h.PulseCount)
	f.Q = make([][]float64, h.PulseCount)
	for p := range f.I {
		f.I[p] = []float64{1, 2, 3, 4, 5, 6, 7, 8}
		f.Q[p] = []float64{-1, -2, -3, -4, -5, -6, -7, -8}
	}
	return f
}

func TestEncodeParseRoundTrip(t *testing.T) {
	orig := validFrame(7)
	raw, err := Encode(orig)
	if err != nil {
		t.Fatalf("encode: %v", err)
	}
	got, err := Parse(bytes.NewReader(raw), ParseConfig{})
	if err != nil {
		t.Fatalf("parse: %v", err)
	}
	if got.Seq != 7 {
		t.Fatalf("seq = %d", got.Seq)
	}
	if got.Header != orig.Header {
		t.Fatalf("header mismatch: %+v vs %+v", got.Header, orig.Header)
	}
	for p := range got.I {
		for s := range got.I[p] {
			if got.I[p][s] != orig.I[p][s] || got.Q[p][s] != orig.Q[p][s] {
				t.Fatalf("sample mismatch at %d,%d", p, s)
			}
		}
	}
}

func TestParseBackToBackStream(t *testing.T) {
	a, _ := Encode(validFrame(0))
	b, _ := Encode(validFrame(1))
	r := bytes.NewReader(append(append([]byte{}, a...), b...))
	for i := int64(0); i < 2; i++ {
		f, err := Parse(r, ParseConfig{})
		if err != nil {
			t.Fatalf("frame %d: %v", i, err)
		}
		if f.Seq != i {
			t.Fatalf("seq = %d want %d", f.Seq, i)
		}
	}
	if _, err := Parse(r, ParseConfig{}); !errors.Is(err, io.EOF) &&
		!errors.Is(err, ErrTruncatedHeader) {
		t.Fatalf("want EOF, got %v", err)
	}
}

func TestParseRejectsBadHeaders(t *testing.T) {
	cases := []struct {
		name   string
		mutate func(h *Header)
		want   error
	}{
		{"sampleRate", func(h *Header) { h.SampleRate = 0 }, ErrInvalidSampleRate},
		{"chirpSlope", func(h *Header) { h.ChirpSlope = -1 }, ErrInvalidChirpSlope},
		{"carrierFreq", func(h *Header) { h.CarrierFreq = 0 }, ErrInvalidCarrierFreq},
		{"pri", func(h *Header) { h.PulseInterval = 0 }, ErrInvalidPRI},
		{"pulseCount", func(h *Header) { h.PulseCount = 0 }, ErrInvalidPulseCount},
		{"samples", func(h *Header) { h.SamplesPerPulse = -2 }, ErrInvalidSamples},
		{"pulseLimit", func(h *Header) { h.PulseCount = 100 }, ErrLimitExceeded},
		{"sampleLimit", func(h *Header) { h.SamplesPerPulse = 100 }, ErrLimitExceeded},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			f := validFrame(0)
			tc.mutate(&f.Header)
			if tc.want == ErrLimitExceeded {
				f.I = make([][]float64, f.Header.PulseCount)
				f.Q = make([][]float64, f.Header.PulseCount)
				for p := range f.I {
					f.I[p] = make([]float64, f.Header.SamplesPerPulse)
					f.Q[p] = make([]float64, f.Header.SamplesPerPulse)
				}
			}
			raw, err := Encode(f)
			if tc.want == ErrLimitExceeded {
				if err != nil {
					t.Fatalf("limit case must encode, got %v", err)
				}
				cfg := ParseConfig{MaxPulseCount: 10, MaxSamplesPerPulse: 10}
				_, err = Parse(bytes.NewReader(raw), cfg)
			}
			if !errors.Is(err, tc.want) {
				t.Fatalf("want %v, got %v", tc.want, err)
			}
		})
	}
}

func TestParseRejectsBadMagicVersionTruncation(t *testing.T) {
	raw, _ := Encode(validFrame(0))

	bad := append([]byte{}, raw...)
	bad[0] = 'X'
	if _, err := Parse(bytes.NewReader(bad), ParseConfig{}); !errors.Is(err, ErrBadMagic) {
		t.Fatalf("magic: %v", err)
	}

	bad = append([]byte{}, raw...)
	bad[8] = 99
	if _, err := Parse(bytes.NewReader(bad), ParseConfig{}); !errors.Is(err, ErrUnsupportedVersion) {
		t.Fatalf("version: %v", err)
	}

	for _, cut := range []int{4, 12, wireHeaderLen, len(raw) - 8} {
		if _, err := Parse(bytes.NewReader(raw[:cut]), ParseConfig{}); err == nil {
			t.Fatalf("cut=%d: expected error", cut)
		}
	}
}

func TestParseRejectsCorruptFrameLen(t *testing.T) {
	raw, _ := Encode(validFrame(0))
	// Declare a header-only frame while a payload actually follows; parser
	// must reject the mismatch without consuming past the frame.
	raw[4], raw[5], raw[6], raw[7] = byte(wireHeaderLen), 0, 0, 0
	if _, err := Parse(bytes.NewReader(raw), ParseConfig{}); !errors.Is(err, ErrPayloadMismatch) {
		t.Fatalf("got %v", err)
	}
}
