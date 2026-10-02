package radar

// ClusterConfig controls peak aggregation.
type ClusterConfig struct {
	// MaxRangeBins/MaxDopplerBins join peaks whose bins differ by at most this
	// value. Values <= 0 default to 1 bin.
	MaxRangeBins   int
	MaxDopplerBins int
}

// Cluster merges adjacent peaks belonging to one target using a union-find
// flood fill. Each cluster outputs one detection: the strongest cell defines
// amplitude/noise while range and velocity are amplitude-weighted centroids.
func Cluster(dets []Detection, rd *RangeDoppler, cfg ClusterConfig) []Detection {
	n := len(dets)
	if n == 0 {
		return nil
	}
	maxR := cfg.MaxRangeBins
	if maxR <= 0 {
		maxR = 1
	}
	maxD := cfg.MaxDopplerBins
	if maxD <= 0 {
		maxD = 1
	}
	nD := len(rd.Magnitude)

	parent := make([]int, n)
	for i := range parent {
		parent[i] = i
	}
	var find func(int) int
	find = func(x int) int {
		for parent[x] != x {
			parent[x] = parent[parent[x]]
			x = parent[x]
		}
		return x
	}
	union := func(a, b int) {
		ra, rb := find(a), find(b)
		if ra != rb {
			parent[rb] = ra
		}
	}
	for i := 0; i < n; i++ {
		for j := i + 1; j < n; j++ {
			dBin := absInt(dets[i].DopplerBin - dets[j].DopplerBin)
			if nD > 0 {
				if wrap := nD - dBin; wrap < dBin {
					dBin = wrap
				}
			}
			rBin := absInt(dets[i].RangeBin - dets[j].RangeBin)
			if dBin <= maxD && rBin <= maxR {
				union(i, j)
			}
		}
	}
	groups := make(map[int][]int)
	for i := 0; i < n; i++ {
		root := find(i)
		groups[root] = append(groups[root], i)
	}
	out := make([]Detection, 0, len(groups))
	roots := make([]int, 0, len(groups))
	for i := 0; i < n; i++ {
		if members, ok := groups[i]; ok && len(members) > 0 && find(members[0]) == i {
			roots = append(roots, i)
		}
	}
	for _, root := range roots {
		members := groups[root]
		strongest := members[0]
		for _, idx := range members[1:] {
			if dets[idx].Amplitude > dets[strongest].Amplitude {
				strongest = idx
			}
		}
		var wSum, rSum, vSum float64
		for _, idx := range members {
			w := dets[idx].Amplitude
			wSum += w
			rSum += w * dets[idx].Range
			vSum += w * dets[idx].Velocity
		}
		merged := dets[strongest]
		merged.Range = rSum / wSum
		merged.Velocity = vSum / wSum
		out = append(out, merged)
	}
	sortDetections(out)
	return out
}

func sortDetections(dets []Detection) {
	for i := 1; i < len(dets); i++ {
		for j := i; j > 0 && lessDetection(dets[j], dets[j-1]); j-- {
			dets[j-1], dets[j] = dets[j], dets[j-1]
		}
	}
}

func lessDetection(a, b Detection) bool {
	if a.DopplerBin != b.DopplerBin {
		return a.DopplerBin < b.DopplerBin
	}
	if a.RangeBin != b.RangeBin {
		return a.RangeBin < b.RangeBin
	}
	return a.Amplitude > b.Amplitude
}
