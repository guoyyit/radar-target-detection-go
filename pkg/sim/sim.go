// Package sim generates deterministic simulated FMCW baseband echo frames for
// testing the processing chain.
package sim

import (
	"errors"
	"math"
	"math/rand/v2"

	"github.com/guoyyit/radar-target-detection-go/pkg/frame"
)

const c0 = 299_792_458.0

// TargetSpec describes one simulated point target. Positive Velocity means
// the target is receding (positive Doppler convention).
type TargetSpec struct {
	Range     float64 // meters at sequence 0
	Velocity  float64 // m/s
	Amplitude float64 // linear amplitude scale
	Phase     float64 // initial phase, radians
}

// Config configures the deterministic generator.
type Config struct {
	Header     frame.Header
	Targets    []TargetSpec
	NoiseSigma float64 // std of each I/Q Gaussian noise component
	DCOffsetI  float64
	DCOffsetQ  float64
	Seed       int64
}

// ErrInvalidConfig reports unusable simulation parameters.
var ErrInvalidConfig = errors.New("sim: invalid configuration")

// Generator produces frames. Frame output depends only on the frame sequence
// number and the seed, so replaying after a restart yields bit-identical data.
type Generator struct {
	cfg Config
}

// New constructs and validates a generator.
func New(cfg Config) (*Generator, error) {
	h := cfg.Header
	switch {
	case h.SampleRate <= 0 || math.IsNaN(h.SampleRate) || math.IsInf(h.SampleRate, 0),
		h.ChirpSlope <= 0 || math.IsNaN(h.ChirpSlope) || math.IsInf(h.ChirpSlope, 0),
		h.CarrierFreq <= 0 || math.IsNaN(h.CarrierFreq) || math.IsInf(h.CarrierFreq, 0),
		h.PulseInterval <= 0 || math.IsNaN(h.PulseInterval) || math.IsInf(h.PulseInterval, 0),
		h.PulseCount <= 0,
		h.SamplesPerPulse <= 0:
		return nil, ErrInvalidConfig
	case cfg.NoiseSigma < 0:
		return nil, ErrInvalidConfig
	}
	return &Generator{cfg: cfg}, nil
}

// Frame generates one baseband frame. Targets advance with time; the noise
// stream is a pure function of (Seed, seq), hence deterministic.
func (g *Generator) Frame(seq int64) (*frame.RawFrame, error) {
	h := g.cfg.Header
	pulses := h.PulseCount
	samples := h.SamplesPerPulse

	iMat := make([][]float64, pulses)
	qMat := make([][]float64, pulses)
	for p := 0; p < pulses; p++ {
		iMat[p] = make([]float64, samples)
		qMat[p] = make([]float64, samples)
	}

	frameTime := float64(seq) * float64(pulses) * h.PulseInterval
	for _, tgt := range g.cfg.Targets {
		if tgt.Amplitude == 0 {
			continue
		}
		for p := 0; p < pulses; p++ {
			slowT := frameTime + float64(p)*h.PulseInterval
			rng := tgt.Range + tgt.Velocity*slowT
			// Doppler shift of the moving target.
			fd := 2 * tgt.Velocity * h.CarrierFreq / c0
			// Beat frequency after deramping: slope*delay minus the Doppler
			// shift (FMCW range-Doppler coupling).
			fb := h.ChirpSlope*2*rng/c0 - fd
			for n := 0; n < samples; n++ {
				fastT := float64(n) / h.SampleRate
				phase := 2*math.Pi*(fb*fastT+fd*slowT) + tgt.Phase
				iMat[p][n] += tgt.Amplitude * math.Cos(phase)
				qMat[p][n] += tgt.Amplitude * math.Sin(phase)
			}
		}
	}

	rng := rand.New(rand.NewPCG(uint64(g.cfg.Seed), uint64(seq)+0x9E3779B97F4A7C15))
	for p := 0; p < pulses; p++ {
		for n := 0; n < samples; n++ {
			iMat[p][n] += g.cfg.DCOffsetI + g.cfg.NoiseSigma*gaussian(rng)
			qMat[p][n] += g.cfg.DCOffsetQ + g.cfg.NoiseSigma*gaussian(rng)
		}
	}

	return &frame.RawFrame{Seq: seq, Header: h, I: iMat, Q: qMat}, nil
}

// EncodedFrame generates and serializes one frame.
func (g *Generator) EncodedFrame(seq int64) ([]byte, error) {
	f, err := g.Frame(seq)
	if err != nil {
		return nil, err
	}
	return frame.Encode(f)
}

// gaussian draws one N(0,1) sample via Box-Muller using the PCG stream.
func gaussian(rng *rand.Rand) float64 {
	u1 := randomUnit(rng)
	u2 := randomUnit(rng)
	return math.Sqrt(-2*math.Log(u1)) * math.Cos(2*math.Pi*u2)
}

func randomUnit(rng *rand.Rand) float64 {
	// Map 53 random bits into (0,1].
	const denom = 1 << 53
	v := (rng.Uint64() >> 11) + 1
	return float64(v) / denom
}
