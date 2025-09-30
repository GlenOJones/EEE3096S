# make_wav_luts.py
import numpy as np
import wave, struct, os
import matplotlib.pyplot as plt

N = 4096          # LUT length, change value of NS in header too
BITS = 12
MAXVAL = (1 << BITS) - 1  # 4095
WAVS = [("piano", "piano.wav"),
        ("guitar", "guitar.wav"),
        ("drum", "drum.wav")]

def read_wav_mono(path):
    with wave.open(path, 'rb') as wf:
        nch = wf.getnchannels()
        nframes = wf.getnframes()
        sw = wf.getsampwidth()
        fr = wf.getframerate()
        raw = wf.readframes(nframes)

    if sw == 1:
        data = np.frombuffer(raw, dtype=np.uint8).astype(np.int16) - 128
        scale = 128.0
    elif sw == 2:
        data = np.frombuffer(raw, dtype='<i2').astype(np.int16)
        scale = 32768.0
    else:
        raise ValueError("Only 8-bit or 16-bit PCM supported.")
    if nch > 1:
        data = data.reshape(-1, nch).mean(axis=1)
    sig = data.astype(np.float64) / scale
    return sig, fr

def to_lut(sig, fr, N=4096):
    """
    Convert audio signal to LUT.
    
    Updated: This function now uses the ENTIRE audio file and resamples it to N points.
    For a 36-second song, all 36 seconds will be compressed into the N-point LUT.
    """
    # Use the entire audio file
    window = sig
    print(f"Original audio: {len(sig)/fr:.2f}s, Using: {len(window)/fr:.2f}s (ENTIRE FILE)")
    
    if len(window) < 4:
        # If absurdly short, tile it
        reps = int(np.ceil(N / max(1, len(window))))
        window = np.tile(window, reps)
    
    # Normalize to [-1, 1] to prevent clipping
    peak = np.max(np.abs(window))
    if peak == 0:
        window = np.zeros_like(window)
        peak = 1.0
    window = window / peak
    
    # Evenly sample N points from the ENTIRE audio file
    # This is the resampling step: original_samples -> N samples
    idx = np.linspace(0, len(window) - 1, N)
    samples = np.interp(idx, np.arange(len(window)), window)
    
    # Map [-1,1] -> [0,4095] with DC offset ~2048
    lut = np.clip((samples * 0.5 + 0.5) * MAXVAL, 0, MAXVAL).round().astype(np.uint16)
    return lut

def c_array(name, arr):
    body = ", ".join(map(str, arr))
    return f"uint16_t {name}[{len(arr)}] = {{ {body} }};"

header_lines = [
    "/* Auto-generated WAV LUTs: 12-bit (0..4095), N=4096 */",
    "#ifndef _EEE3096S_PRAC4_WAV_LUTS_H_",
    "#define _EEE3096S_PRAC4_WAV_LUTS_H_",
    "#include <stdint.h>",
    "#define NS 4096",
    "#define FSIGNAL    1u           // Play entire LUT once per second",
    "#define FSIGNAL_PIANO   0.053f  // 1/18.86 for original piano duration",
    "#define FSIGNAL_GUITAR  0.095f  // 1/10.58 for original guitar duration  ",
    "#define FSIGNAL_DRUM    0.088f  // 1/11.34 for original drum duration",
    "// This gives: TIM2_TICKS = 16,000,000 / (4096 × 1) = 3906",
]

for key, fname in WAVS:
    if not os.path.exists(fname):
        print(f"[WARN] '{fname}' not found – skipping this LUT.")
        continue
    sig, fr = read_wav_mono(fname)
    lut = to_lut(sig, fr, N)
    # Plot for verification
    plt.figure()
    plt.plot(lut)
    plt.title(f"{key.title()} LUT (N={N})")
    plt.xlabel("Sample index")
    plt.ylabel("Value (0..4095)")
    plt.grid(True)
    plt.tight_layout()
    # Add to header
    header_lines.append(c_array(f"{key}_wav_LUT", lut))

header_lines.append("#endif /* _EEE3096S_PRAC4_WAV_LUTS_H_ */")

with open("EEE3096S_Prac4_WAV_LUTS.h", "w") as f:
    f.write("\n".join(header_lines) + "\n")

plt.show()
print("Generated: EEE3096S_Prac4_WAV_LUTS.h")
