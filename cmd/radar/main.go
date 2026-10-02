package main

import (
	"context"
	"flag"
	"fmt"

	"github.com/guoyyit/radar-target-detection-go/pkg/radar"
)

func defaultConfig() radar.Config {
	return radar.Config{
		Parse: radar.ParseConfig{
			MinPulses: 4, MaxPulses: 128,
			MinSamples: 16, MaxSamples: 4096,
			MinSampleRateHz: 1, MinCarrierHz: 1, MinPRISec: 1e-9,
		},
		Spectrum: radar.SpectrumConfig{
			Window: radar.WindowConfig{
				RangeWindow:   radar.WindowHann,
				DopplerWindow: radar.WindowHann,
				RemoveDC:      true,
			},
		},
		CFAR: radar.CFARConfig{
			GuardRange: 1, GuardDoppler: 1,
			TrainRange: 6, TrainDoppler: 3,
			Alpha: 8, ThresholdDB: 13, SkipDCBins: 2,
		},
		Cluster: radar.ClusterConfig{MaxRangeBins: 2, MaxDopplerBins: 1},
		Tracker: radar.TrackerConfig{
			AssocRange: 2, AssocVelocity: 1.5,
			ConfirmCount: 2, HoldFrames: 3, WindowSize: 3,
		},
	}
}

func main() {
	checkpoint := flag.String("checkpoint", "checkpoint.json", "checkpoint file path (empty disables)")
	frames := flag.Int("frames", 8, "number of frames to simulate")
	flag.Parse()

	var store radar.CheckpointStore = radar.NewMemoryCheckpointStore()
	var err error
	if *checkpoint != "" {
		store, err = radar.NewFileCheckpointStore(*checkpoint)
		if err != nil {
			panic(err)
		}
	}
	proc, err := radar.NewProcessor(defaultConfig(), store)
	if err != nil {
		panic(err)
	}

	pipe := radar.NewPipe(4)
	go func() {
		defer pipe.Close()
		header := radar.FrameHeader{
			SampleRateHz: 20e6, ChirpSlope: 20e12,
			PulseCount: 32, SamplesPerPulse: 256,
			CarrierFreqHz: 77e9, PRISec: 20e-6,
		}
		targets := []radar.SimTarget{
			{Range: 12, Velocity: 3.5, Amplitude: 8},
			{Range: 30, Velocity: -6.0, Amplitude: 6},
		}
		for id := int64(1); id <= int64(*frames); id++ {
			f := radar.SimulateFrame(radar.SimConfig{
				Header: header, Targets: targets,
				NoiseSigma: 0.5, DCOffsetI: 2, DCOffsetQ: -1,
				Seed: 1000 + id,
			})
			f.ID = id
			if !pipe.Push(f) {
				return
			}
		}
	}()

	emit := func(r radar.FrameResult) {
		if r.Rejected {
			fmt.Printf("frame %d rejected: %d\n", r.FrameID, r.Reason)
			return
		}
		fmt.Printf("frame %d: %d detections, %d targets\n", r.FrameID, len(r.Detections), len(r.Targets))
		for _, t := range r.Targets {
			fmt.Printf("  track %2d  R=%7.2fm  v=%7.2fm/s  A=%6.2f  SNR=%5.1fdB\n",
				t.TrackID, t.Range, t.Velocity, t.Amplitude, t.SNRDB)
		}
	}
	if err := proc.Run(context.Background(), pipe, emit); err != nil {
		panic(err)
	}
}
