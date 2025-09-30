# make_luts.py
import numpy as np
import matplotlib.pyplot as plt

N = 4096  # Changed to match WAV LUTs - now 4096 points for consistency
MAXVAL = 4095

i = np.arange(N)

# Sine 0..4095 (centered around 2048)
sine = np.round((np.sin(2*np.pi*i/N) * 0.5 + 0.5) * MAXVAL).astype(int)

# Sawtooth 0..4095
saw = np.round(i * (MAXVAL) / (N - 1)).astype(int)

# Triangle 0..4095 (rise then fall)
half = N // 2
tri = np.concatenate([
    np.linspace(0, MAXVAL, half, endpoint=True),
    np.linspace(MAXVAL, 0, N - half, endpoint=True),
]).round().astype(int)

def c_array(name, arr):
    body = ", ".join(map(str, arr))
    return f"uint16_t {name}[{len(arr)}] = {{ {body} }};"

# Print C arrays to paste into main.c
print(c_array("sine_LUT", sine))
print()
print(c_array("saw_LUT", saw))
print()
print(c_array("tri_LUT", tri))

# Quick plots (one per figure, no special colors)
for data, title in [(sine, "Sine LUT"), (saw, "Sawtooth LUT"), (tri, "Triangle LUT")]:
    plt.figure()
    plt.plot(data)
    plt.title(title)
    plt.xlabel("Sample index")
    plt.ylabel("Value (0..4095)")
    plt.grid(True)
    plt.tight_layout()

plt.show()
