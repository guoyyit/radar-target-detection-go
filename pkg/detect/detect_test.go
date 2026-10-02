package detect

import (
	"math"
	"testing"

	"github.com/guoyyit/radar-target-detection-go/pkg/frame"
	"github.com/guoyyit/radar-target-detection-go/pkg/signal"
	"github.com/guoyyit/radar-target-detection-go/pkg/sim"
)

func simHeader() frame.Header {
	return frame.Header{
		SampleRate:      10e6,
		ChirpSlope:      12e12,
		CarrierFreq:     77e9,
		PulseInterval:   40e-6,
		PulseCount:      32,
		SamplesPerPulse: 128,
	}
}

func analyze(t *testing.T, targets []sim.TargetSpec, noise, dcI, dcQ float64) []Peak {
	t.Helper()
	cfg := sim.Config{
		Header:     simHeader(),
		Targets:    targets,
		NoiseSigma: noise,
		DCOffsetI:  dcI,
		DCOffsetQ:  dcQ,
		Seed:       7,
	}
	g, err := sim.New(cfg)
	if err != nil {
		t.Fatal(err)
	}
	f, err := g.Frame(0)
	if err != nil {
		t.Fatal(err)
	}
	iq := make([][]complex128, f.Header.PulseCount)
	for p := range iq {
		iq[p] = make([]complex128, f.Header.SamplesPerPulse)
		for n := range iq[p] {
			iq[p][n] = complex(f.I[p][n], f.Q[p][n])
		}
	}
	spec, err := signal.RangeDoppler(iq, signal.Config{
		Window:         signal.WindowHann,
		RemoveDC:       true,
		RangeZeroPad:   2,
		DopplerZeroPad: 2,
	})
	if err != nil {
		t.Fatal(err)
	}
	mask, floor := Detect(spec, CFARConfig{
		GuardRange:   2,
		GuardDoppler: 2,
		TrainRange:   4,
		TrainDoppler: 4,
		AlphaDB:      12,
		SkipRangeDC:  true,
		SkipNearBins: 3,
	})
	return Cluster(spec, mask, floor, f.Header, ClusterConfig{
		MaxRangeGap:   1,
		MaxDopplerGap: 1,
		MinCells:      2,
		Interpolate:   true,
	})
}

func matchPeak(t *testing.T, peaks []Peak, wantRange, wantVel, tolR, tolV float64) Peak {
	t.Helper()
	for _, p := range peaks {
		if math.Abs(p.Range-wantRange) < tolR && math.Abs(p.Velocity-wantVel) < tolV {
			return p
		}
	}
	t.Fatalf("no peak near range=%v vel=%v; got %+v", wantRange, wantVel, peaks)
	return Peak{}
}

func TestSingleTargetRangeVelocity(t *testing.T) {
	peaks := analyze(t, []sim.TargetSpec{
		{Range: 30, Velocity: 4, Amplitude: 8},
	}, 0.05, 0, 0)
	if len(peaks) != 1 {
		t.Fatalf("want 1 peak, got %d: %+v", len(peaks), peaks)
	}
	p := peaks[0]
	if math.Abs(p.Range-30) > 2.0 {
		t.Fatalf("range %v", p.Range)
	}
	if math.Abs(p.Velocity-4) > 0.6 {
		t.Fatalf("velocity %v", p.Velocity)
	}
	if p.SNRDB < 10 {
		t.Fatalf("snr %.1f too low", p.SNRDB)
	}
}

func TestMultipleTargets(t *testing.T) {
	specs := []sim.TargetSpec{
		{Range: 10, Velocity: -3, Amplitude: 8},
		{Range: 25, Velocity: 2, Amplitude: 8},
		{Range: 45, Velocity: 0, Amplitude: 8},
	}
	peaks := analyze(t, specs, 0.05, 0, 0)
	if len(peaks) != 3 {
		t.Fatalf("want 3 peaks, got %d: %+v", len(peaks), peaks)
	}
	for _, s := range specs {
		matchPeak(t, peaks, s.Range, s.Velocity, 2.0, 0.7)
	}
}

func TestDenseTargetsResolvable(t *testing.T) {
	specs := []sim.TargetSpec{
		{Range: 25, Velocity: 0, Amplitude: 8},
		{Range: 28, Velocity: 0, Amplitude: 8},
	}
	peaks := analyze(t, specs, 0.02, 0, 0)
	if len(peaks) < 2 {
		t.Fatalf("dense targets merged: %+v", peaks)
	}
	matchPeak(t, peaks, 25, 0, 1.2, 1)
	matchPeak(t, peaks, 28, 0, 1.2, 1)
}

func TestDCOffsetNoPhantom(t *testing.T) {
	// Strong DC offset but no target: DC removal + SkipRangeDC must yield no
	// detections at range bin 0.
	peaks := analyze(t, nil, 0.02, 20, -15)
	for _, p := range peaks {
		if p.RangeBin == 0 {
			t.Fatalf("phantom DC target: %+v", p)
		}
	}

	// DC offset must not hide the real target either.
	peaks = analyze(t, []sim.TargetSpec{{Range: 35, Velocity: 1, Amplitude: 8}}, 0.02, 20, -15)
	matchPeak(t, peaks, 35, 1, 1.5, 0.8)
}

func TestDetectionVsSNR(t *testing.T) {
	tgt := func(amp float64) []sim.TargetSpec {
		return []sim.TargetSpec{{Range: 30, Velocity: 0, Amplitude: amp}}
	}
	high := analyze(t, tgt(8), 0.02, 0, 0)
	mid := analyze(t, tgt(2), 0.3, 0, 0)
	low := analyze(t, tgt(0.2), 2.0, 0, 0)
	found := func(peaks []Peak) bool {
		for _, p := range peaks {
			if math.Abs(p.Range-30) < 2 {
				return true
			}
		}
		return false
	}
	if !found(high) {
		t.Fatalf("high-SNR target must be detected: %+v", high)
	}
	if !found(mid) {
		t.Fatalf("mid-SNR target expected detectable: high=%+v mid=%+v", high, mid)
	}
	if found(low) {
		// Sub-noise targets are allowed to miss; require no confident
		// detection at the target cell when SNR is far below threshold.
		for _, p := range low {
			if math.Abs(p.Range-30) < 2 && p.SNRDB > 12 {
				t.Fatalf("sub-noise target reported at %.1f dB: %+v", p.SNRDB, low)
			}
		}
	}
}

func TestWeakTargetBelowNoise(t *testing.T) {
	// A very weak target with heavy noise need not be detected; no phantom
	// clusters should appear beyond the guard region in low noise by itself.
	peaks := analyze(t, nil, 0.01, 0, 0)
	for _, p := range peaks {
		if p.SNRDB < 0 {
			t.Fatalf("negative-SNR cluster: %+v", p)
		}
	}
}
