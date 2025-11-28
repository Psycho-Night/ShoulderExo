import struct
import numpy as np
import matplotlib.pyplot as plt
import os

# === File path ===
folder = r"Files\Tunning0_05Hz"

plot_folder = os.path.join(folder, "Plots")
os.makedirs(plot_folder, exist_ok=True)

file_number = 7

for n in range(1,file_number+1):
    # Check file exists
    filename = f"AAN_{n:04d}.bin"
    filepath = os.path.join(folder, filename)

    if not os.path.exists(filepath):
        raise FileNotFoundError(f"File not found: {filepath}")

    frame_size = 40   # bytes

    # Read entire file
    with open(filepath, "rb") as f:
        data = f.read()

    file_size = len(data)
    num_frames = file_size // frame_size

    # Struct format: little-endian (<), matches:
    # uint16, uint32, 8x float32, uint16
    fmt = "<H I f f f f f f f f H"
    frame = struct.Struct(fmt)

    if frame.size != frame_size:
        raise ValueError("Struct size mismatch. Your firmware struct is wrong.")

    # Preallocate numpy arrays
    Header           = np.zeros(num_frames, dtype=np.uint16)
    Time             = np.zeros(num_frames, dtype=np.uint32)
    Target_Angle     = np.zeros(num_frames, dtype=np.float32)
    Current_Angle    = np.zeros(num_frames, dtype=np.float32)
    Tau_total        = np.zeros(num_frames, dtype=np.float32)
    Tau_ff           = np.zeros(num_frames, dtype=np.float32)
    Tau_fb           = np.zeros(num_frames, dtype=np.float32)
    Desired_Current  = np.zeros(num_frames, dtype=np.float32)
    Actual_Current   = np.zeros(num_frames, dtype=np.float32)
    SysFreq          = np.zeros(num_frames, dtype=np.float32)
    Padding          = np.zeros(num_frames, dtype=np.uint16)

    # Parse frames
    for i in range(num_frames):
        start = i * frame_size
        fields = frame.unpack_from(data, start)

        Header[i]          = fields[0]
        Time[i]            = fields[1]
        Target_Angle[i]    = fields[2]
        Current_Angle[i]   = fields[3]
        Tau_total[i]       = fields[4]
        Tau_ff[i]          = fields[5]
        Tau_fb[i]          = fields[6]
        Desired_Current[i] = fields[7]
        Actual_Current[i]  = fields[8]
        SysFreq[i]         = fields[9]
        Padding[i]         = fields[10]

    # === Quick checks ===
    # print("Unique Header values:", np.unique(Header))
    # print("Unique Padding values:", np.unique(Padding))

    # === Plot ===
    plt.figure(figsize=(10, 6))

    plt.subplot(4,1,1)
    plt.plot(Time * 1e-6, Target_Angle, label="Target Angle")
    plt.plot(Time * 1e-6, Current_Angle, label="Current Angle")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.legend()

    plt.subplot(4,1,2)
    plt.plot(Time * 1e-6, Tau_fb, label = "FB")
    plt.plot(Time * 1e-6, Tau_ff, label="FF")
    plt.plot(Time * 1e-6, Tau_total, label="Total")
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (Nm)")
    plt.legend()

    plt.subplot(4,1,3)
    plt.plot(Time * 1e-6, Desired_Current, label = "Current Controller")
    plt.plot(Time * 1e-6, Actual_Current, label="Current ESCON")
    plt.xlabel("Time (s)")
    plt.ylabel("Current (A)")
    plt.legend()

    plt.subplot(4,1,4)
    plt.plot(Time * 1e-6, SysFreq, label = "System Frequency")
    plt.xlabel("Time (s)")
    plt.ylabel("Frequency (Hz)")
    plt.legend()

    plt.tight_layout()
    
    out_path = os.path.join(plot_folder, f"Iteration_{n:04d}.png")
    plt.savefig(os.path.join(plot_folder, f"Iteration_{n:04d}.png"), dpi=300)


plt.show()