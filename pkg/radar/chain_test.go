package radar

import (
	"math"
	"testing"
)

func testHeader() FrameHeader {
	return FrameHeader{
		SampleRateHz:    20e6,
		ChirpSlope:      20e12,
		PulseCount:      32,
		SamplesPerPulse: 256,
		CarrierFreqHz:   77e9,
		PRISec:          20e-6,
	}
}

func testConfig() Config {
	return Config{
		Parse: ParseConfig{
			MinPulses: 4, MaxPulses: 128,
			MinSamples: 16, MaxSamples: 4096,
			MinSampleRateHz: 1, MinCarrierHz: 1, MinPRISec: 1e-9,
		},
		Spectrum: SpectrumConfig{Window: WindowConfig{
			RangeWindow: WindowHann, DopplerWindow: WindowHann,
			RemoveDC: true, RangePadTo: 256,
		}},
		CFAR: CFARConfig{
			GuardRange: 1, GuardDoppler: 1,
			TrainRange: 6, TrainDoppler: 3,
			Alpha: 5, ThresholdDB: 10, SkipDCBins: 1,
		},
		Cluster: ClusterConfig{MaxRangeBins: 2, MaxDopplerBins: 1},
		Tracker: TrackerConfig{
			AssocRange: 2.5, AssocVelocity: 2.0,
			ConfirmCount: 2, HoldFrames: 2, WindowSize: 3,
		},
	}
}

func processRaw(t *testing.T, cfg Config, raw RawFrame) *FrameResult {
	t.Helper()
	p, err := NewProcessor(cfg, NewMemoryCheckpointStore())
	if err != nil {
		t.Fatal(err)
	}
	res, err := p.ProcessFrame(raw)
	if err != nil {
		t.Fatal(err)
	}
	return res
}

func TestMultiTargetDetection(t *testing.T) {
	targets := []SimTarget{
		{Range: 12, Velocity: 3.5, Amplitude: 10},
		{Range: 30, Velocity: -6.0, Amplitude: 8},
	}
	raw := SimulateFrame(SimConfig{
		Header: testHeader(), Targets: targets,
		NoiseSigma: 0.4, Seed: 42,
	})
	raw.ID = 1
	res := processRaw(t, testConfig(), raw)
	if res.Rejected {
		t.Fatalf("frame rejected: %v", res.Reason)
	}
	for _, want := range targets {
		found := false
		for _, d := range res.Detections {
			if math.Abs(d.Range-want.Range) < 1.5 && math.Abs(d.Velocity-want.Velocity) < 1.2 && d.SNRDB > 10 {
				found = true
			}
		}
		if !found {
			t.Errorf("target R=%g v=%g not found among %d detections: %+v",
				want.Range, want.Velocity, len(res.Detections), res.Detections)
		}
	}
	if len(res.Detections) > 8 {
		t.Errorf("too many detections, expected <=8 got %d", len(res.Detections))
	}
}

func TestDenseTargetsSeparated(t *testing.T) {
	targets := []SimTarget{
		{Range: 20, Velocity: 2.0, Amplitude: 10},
		{Range: 24, Velocity: 2.0, Amplitude: 10}, // ~2 range bins apart
	}
	raw := SimulateFrame(SimConfig{
		Header: testHeader(), Targets: targets,
		NoiseSigma: 0.3, Seed: 7,
	})
	raw.ID = 1
	res := processRaw(t, testConfig(), raw)
	count := 0
	for _, d := range res.Detections {
		if math.Abs(d.Velocity-2) < 1.5 && d.Range > 17 && d.Range < 27 && d.SNRDB > 10 {
			count++
		}
	}
	if count < 2 {
		t.Fatalf("dense targets merged, got %d distinct detections: %+v", count, res.Detections)
	}
}

func TestDCOffsetNoFalseTarget(t *testing.T) {
	raw := SimulateFrame(SimConfig{
		Header:     testHeader(),
		Targets:    nil,
		NoiseSigma: 0.3,
		DCOffsetI:  50,
		DCOffsetQ:  -30,
		Seed:       99,
	})
	raw.ID = 1
	cfg := testConfig()
	cfg.CFAR.ThresholdDB = 12
	res := processRaw(t, cfg, raw)
	for _, d := range res.Detections {
		if d.Range < 2.5 {
			t.Errorf("false target near DC bin: %+v", d)
		}
	}
	if len(res.Detections) > 2 {
		t.Errorf("DC-only frame produced %d detections: %+v", len(res.Detections), res.Detections)
	}
}

func TestSNRScenarios(t *testing.T) {
	cfg := testConfig()
	for _, amp := range []float64{0.6, 4, 12} {
		raw := SimulateFrame(SimConfig{
			Header:     testHeader(),
			Targets:    []SimTarget{{Range: 25, Velocity: 1, Amplitude: amp}},
			NoiseSigma: 1, Seed: 5,
		})
		raw.ID = 1
		res := processRaw(t, cfg, raw)
		maxSNR := math.Inf(-1)
		for _, d := range res.Detections {
			if d.SNRDB > maxSNR {
				maxSNR = d.SNRDB
			}
		}
		if amp > 4 && maxSNR < 12 {
			t.Errorf("strong target amp=%g not detected, maxSNR=%v", amp, maxSNR)
		}
	}
}

func TestReproducibleSpectrum(t *testing.T) {
	cfg := testConfig()
	raw := SimulateFrame(SimConfig{
		Header: testHeader(),
		Targets: []SimTarget{
			{Range: 15, Velocity: 4, Amplitude: 7},
			{Range: 40, Velocity: -3, Amplitude: 5},
		},
		NoiseSigma: 0.8, DCOffsetI: 3, Seed: 123,
	})
	frame, err := ParseFrame(raw, cfg.Parse)
	if err != nil {
		t.Fatal(err)
	}
	rd1, err := ComputeRangeDoppler(frame, cfg.Spectrum)
	if err != nil {
		t.Fatal(err)
	}
	rd2, err := ComputeRangeDoppler(frame, cfg.Spectrum)
	if err != nil {
		t.Fatal(err)
	}
	for d := range rd1.Magnitude {
		for r := range rd1.Magnitude[d] {
			if rd1.Magnitude[d][r] != rd2.Magnitude[d][r] {
				t.Fatalf("spectrum not reproducible at (%d,%d)", d, r)
			}
		}
	}
}

func TestSimulationDeterministic(t *testing.T) {
	cfg := SimConfig{
		Header: testHeader(),
		Targets: []SimTarget{
			{Range: 10, Velocity: 2, Amplitude: 5},
		},
		NoiseSigma: 1, DCOffsetI: 2, Seed: 2024,
	}
	a := SimulateFrame(cfg)
	b := SimulateFrame(cfg)
	for p := range a.Pulses {
		for k := range a.Pulses[p].I {
			if a.Pulses[p].I[k] != b.Pulses[p].I[k] || a.Pulses[p].Q[k] != b.Pulses[p].Q[k] {
				t.Fatalf("simulation differs at pulse %d sample %d", p, k)
			}
		}
	}
}
