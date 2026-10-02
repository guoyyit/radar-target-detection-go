package radar

import "math"

// SimTarget describes one simulated point target.
type SimTarget struct {
	Range     float64 // metres
	Velocity  float64 // metres per second (positive = receding)
	Amplitude float64 // linear amplitude
}

// SimConfig holds deterministic simulation parameters.
type SimConfig struct {
	Header     FrameHeader
	Targets    []SimTarget
	NoiseSigma float64
	DCOffsetI  float64
	DCOffsetQ  float64
	Seed       int64
}

// SimulateFrame generates one deterministic baseband FMCW echo frame.
// The signal model per target is
//
//	x[p,k] = A exp(j 2pi (fb*k/fs + fd*(p*PRI + k/fs)))
//
// with fb = slope*2R/c and fd = 2v/lambda. Complex Gaussian noise and a
// constant DC offset are added. Identical (seed, config) always produces
// byte-identical samples.
func SimulateFrame(cfg SimConfig) RawFrame {
	h := cfg.Header
	lambda := speedOfLight / h.CarrierFreqHz
	rng := newPCG(cfg.Seed)
	gauss := func() float64 {
		u1 := 0.0
		for u1 == 0 {
			u1 = rng.float()
		}
		return math.Sqrt(-2*math.Log(u1)) * math.Cos(2*math.Pi*rng.float())
	}

	pulses := make([]Pulse, h.PulseCount)
	for p := 0; p < h.PulseCount; p++ {
		pulse := Pulse{
			I: make([]float64, h.SamplesPerPulse),
			Q: make([]float64, h.SamplesPerPulse),
		}
		slow := float64(p) * h.PRISec
		for k := 0; k < h.SamplesPerPulse; k++ {
			iVal := cfg.DCOffsetI + cfg.NoiseSigma*gauss()
			qVal := cfg.DCOffsetQ + cfg.NoiseSigma*gauss()
			t := float64(k) / h.SampleRateHz
			for _, tgt := range cfg.Targets {
				fb := h.ChirpSlope * 2 * tgt.Range / speedOfLight
				fd := 2 * tgt.Velocity / lambda
				phase := 2 * math.Pi * (fb*t + fd*(slow+t))
				iVal += tgt.Amplitude * math.Cos(phase)
				qVal += tgt.Amplitude * math.Sin(phase)
			}
			pulse.I[k] = iVal
			pulse.Q[k] = qVal
		}
		pulses[p] = pulse
	}
	return RawFrame{
		ID:     1,
		Header: h,
		Pulses: pulses,
	}
}

// pcg32 is a deterministic permuted congruential generator (PCG-XSH-RR 64/32),
// used to derive stable pseudo-random bits independently of the global RNG.
type pcg32 struct {
	state uint64
	inc   uint64
}

func newPCG(seed int64) *pcg32 {
	p := &pcg32{inc: 1442695040888963407 | 1}
	p.state = 0
	p.next()
	p.state += uint64(seed)
	p.next()
	return p
}

func (p *pcg32) next() uint32 {
	old := p.state
	p.state = old*6364136223846793005 + p.inc
	xorshifted := uint32(((old >> 18) ^ old) >> 27)
	rot := uint32(old >> 59)
	return (xorshifted >> rot) | (xorshifted << ((-rot) & 31))
}

func (p *pcg32) float() float64 {
	return float64(p.next()>>8) / float64(1<<24)
}
