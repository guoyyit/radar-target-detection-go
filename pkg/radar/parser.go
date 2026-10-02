package radar

import (
	"errors"
	"fmt"
)

// ParseErrorKind enumerates the distinguishable rejection reasons.
type ParseErrorKind int

const (
	ParseOK ParseErrorKind = iota
	ParseEmptyFrame
	ParseBadSampleRate
	ParseBadChirpSlope
	ParseBadPulseCount
	ParseBadSamplesPerPulse
	ParseBadCarrierFreq
	ParseBadPRI
	ParsePulseCountMismatch
	ParseSampleCountMismatch
	ParseParameterMismatch
	ParseParameterOutOfLimit
)

// ParseError describes why a frame was rejected.
type ParseError struct {
	Kind ParseErrorKind
	Msg  string
}

func (e *ParseError) Error() string {
	if e == nil {
		return ""
	}
	return e.Msg
}

// ParseConfig defines accepted parameter limits and expected dimensions.
type ParseConfig struct {
	MinSampleRateHz float64
	MaxSampleRateHz float64
	MinChirpSlope   float64
	MaxChirpSlope   float64
	MinPulses       int
	MaxPulses       int
	MinSamples      int
	MaxSamples      int
	MinCarrierHz    float64
	MaxCarrierHz    float64
	MinPRISec       float64
	MaxPRISec       float64
	// ExpectedPulses/ExpectedSamples > 0 require the frame to match exactly.
	ExpectedPulses  int
	ExpectedSamples int
}

var ErrFrameRejected = errors.New("frame rejected")

// ParseFrame validates raw against cfg and returns a validated Frame.
func ParseFrame(raw RawFrame, cfg ParseConfig) (*Frame, error) {
	h := raw.Header
	if len(raw.Pulses) == 0 {
		return nil, &ParseError{Kind: ParseEmptyFrame, Msg: "frame contains no pulses"}
	}
	checks := []struct {
		kind ParseErrorKind
		ok   bool
		msg  string
	}{
		{ParseBadSampleRate, h.SampleRateHz > 0 && !(cfg.MinSampleRateHz > 0 && h.SampleRateHz < cfg.MinSampleRateHz) && !(cfg.MaxSampleRateHz > 0 && h.SampleRateHz > cfg.MaxSampleRateHz),
			fmt.Sprintf("sample rate %g out of range [%g,%g]", h.SampleRateHz, cfg.MinSampleRateHz, cfg.MaxSampleRateHz)},
		{ParseBadChirpSlope, h.ChirpSlope != 0 && !(cfg.MinChirpSlope > 0 && h.ChirpSlope < cfg.MinChirpSlope) && !(cfg.MaxChirpSlope > 0 && h.ChirpSlope > cfg.MaxChirpSlope),
			fmt.Sprintf("chirp slope %g out of range [%g,%g]", h.ChirpSlope, cfg.MinChirpSlope, cfg.MaxChirpSlope)},
		{ParseBadPulseCount, inRange(h.PulseCount, cfg.MinPulses, cfg.MaxPulses),
			fmt.Sprintf("pulse count %d out of range [%d,%d]", h.PulseCount, cfg.MinPulses, cfg.MaxPulses)},
		{ParseBadSamplesPerPulse, h.SamplesPerPulse > 0 && inRange(h.SamplesPerPulse, cfg.MinSamples, cfg.MaxSamples),
			fmt.Sprintf("samples per pulse %d out of range [%d,%d]", h.SamplesPerPulse, cfg.MinSamples, cfg.MaxSamples)},
		{ParseBadCarrierFreq, h.CarrierFreqHz > 0 && !(cfg.MinCarrierHz > 0 && h.CarrierFreqHz < cfg.MinCarrierHz) && !(cfg.MaxCarrierHz > 0 && h.CarrierFreqHz > cfg.MaxCarrierHz),
			fmt.Sprintf("carrier frequency %g out of range [%g,%g]", h.CarrierFreqHz, cfg.MinCarrierHz, cfg.MaxCarrierHz)},
		{ParseBadPRI, h.PRISec > 0 && !(cfg.MinPRISec > 0 && h.PRISec < cfg.MinPRISec) && !(cfg.MaxPRISec > 0 && h.PRISec > cfg.MaxPRISec),
			fmt.Sprintf("PRI %g out of range [%g,%g]", h.PRISec, cfg.MinPRISec, cfg.MaxPRISec)},
	}
	for _, c := range checks {
		if !c.ok {
			return nil, &ParseError{Kind: c.kind, Msg: c.msg}
		}
	}
	if cfg.ExpectedPulses > 0 && h.PulseCount != cfg.ExpectedPulses {
		return nil, &ParseError{
			Kind: ParseParameterMismatch,
			Msg:  fmt.Sprintf("pulse count %d does not match expected %d", h.PulseCount, cfg.ExpectedPulses),
		}
	}
	if cfg.ExpectedSamples > 0 && h.SamplesPerPulse != cfg.ExpectedSamples {
		return nil, &ParseError{
			Kind: ParseParameterMismatch,
			Msg:  fmt.Sprintf("samples per pulse %d does not match expected %d", h.SamplesPerPulse, cfg.ExpectedSamples),
		}
	}
	if len(raw.Pulses) != h.PulseCount {
		return nil, &ParseError{
			Kind: ParsePulseCountMismatch,
			Msg:  fmt.Sprintf("declared %d pulses but payload has %d", h.PulseCount, len(raw.Pulses)),
		}
	}
	for i, p := range raw.Pulses {
		if len(p.I) != h.SamplesPerPulse || len(p.Q) != h.SamplesPerPulse {
			return nil, &ParseError{
				Kind: ParseSampleCountMismatch,
				Msg: fmt.Sprintf("pulse %d carries %d/%d samples, expected %d",
					i, len(p.I), len(p.Q), h.SamplesPerPulse),
			}
		}
	}
	return &Frame{ID: raw.ID, Header: h, Pulses: raw.Pulses}, nil
}

func inRange(v, lo, hi int) bool {
	if lo > 0 && v < lo {
		return false
	}
	if hi > 0 && v > hi {
		return false
	}
	return v > 0
}
