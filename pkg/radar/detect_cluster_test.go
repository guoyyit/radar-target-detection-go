package radar

import "testing"

func syntheticRD(nD, nR int) *RangeDoppler {
	mag := make([][]float64, nD)
	for d := range mag {
		mag[d] = make([]float64, nR)
		for r := range mag[d] {
			mag[d][r] = 1
		}
	}
	rAxis := make([]float64, nR)
	dAxis := make([]float64, nD)
	for r := range rAxis {
		rAxis[r] = float64(r)
	}
	for d := range dAxis {
		dAxis[d] = float64(d)
	}
	return &RangeDoppler{Magnitude: mag, RangeAxis: rAxis, DopplerAxis: dAxis}
}

func TestDetectFindsPeakAndIgnoresNoise(t *testing.T) {
	rd := syntheticRD(16, 64)
	rd.Magnitude[5][20] = 100
	cfg := CFARConfig{
		GuardRange: 1, GuardDoppler: 1,
		TrainRange: 3, TrainDoppler: 2,
		Alpha: 3, ThresholdDB: 10, SkipDCBins: 1,
		MinTrainingCells: 4,
	}
	dets := Detect(rd, cfg)
	if len(dets) != 1 {
		t.Fatalf("want 1 detection got %d: %+v", len(dets), dets)
	}
	if dets[0].RangeBin != 20 || dets[0].DopplerBin != 5 {
		t.Fatalf("peak at wrong cell: %+v", dets[0])
	}
	if dets[0].SNRDB <= 30 {
		t.Fatalf("isolated peak SNR too low: %v", dets[0].SNRDB)
	}
}

func TestDetectSeparatesAdjacentPeaks(t *testing.T) {
	rd := syntheticRD(16, 64)
	rd.Magnitude[5][20] = 100
	rd.Magnitude[5][21] = 100
	cfg := CFARConfig{
		GuardRange: 0, GuardDoppler: 0,
		TrainRange: 3, TrainDoppler: 2,
		Alpha: 3, ThresholdDB: 10, SkipDCBins: 1,
	}
	dets := Detect(rd, cfg)
	if len(dets) != 2 {
		t.Fatalf("adjacent equal peaks must both be local maxima, got %d: %+v", len(dets), dets)
	}
}

func TestClusterMergesAdjacentKeepsFar(t *testing.T) {
	rd := syntheticRD(16, 64)
	dets := []Detection{
		{RangeBin: 20, DopplerBin: 5, Range: 20, Velocity: 5, Amplitude: 10, SNRDB: 30},
		{RangeBin: 21, DopplerBin: 5, Range: 21, Velocity: 5, Amplitude: 6, SNRDB: 25},
		{RangeBin: 40, DopplerBin: 8, Range: 40, Velocity: 8, Amplitude: 9, SNRDB: 28},
	}
	out := Cluster(dets, rd, ClusterConfig{MaxRangeBins: 1, MaxDopplerBins: 1})
	if len(out) != 2 {
		t.Fatalf("want 2 clusters got %d: %+v", len(out), out)
	}
	// The strong member defines amplitude; centroid weights range.
	var merged Detection
	for _, d := range out {
		if d.Amplitude == 10 {
			merged = d
		}
	}
	if merged.Amplitude != 10 || merged.RangeBin != 20 || merged.Range <= 20 || merged.Range >= 21 {
		t.Fatalf("merged cluster wrong: %+v", merged)
	}
}

func TestClusterDopplerWrap(t *testing.T) {
	rd := syntheticRD(16, 64)
	dets := []Detection{
		{RangeBin: 10, DopplerBin: 0, Range: 10, Velocity: 0, Amplitude: 5},
		{RangeBin: 10, DopplerBin: 15, Range: 10, Velocity: 0, Amplitude: 5},
	}
	out := Cluster(dets, rd, ClusterConfig{MaxRangeBins: 1, MaxDopplerBins: 1})
	if len(out) != 1 {
		t.Fatalf("doppler wrap bins 0/15 should merge, got %d", len(out))
	}
}
