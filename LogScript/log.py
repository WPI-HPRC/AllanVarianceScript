import csv
import time
import serial

PORT = "COM5"
BAUD = 921600
OUTFILE = "allan_log.csv"
DURATION_S = 7 * 3600

def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    #clear junk
    ser.reset_input_buffer()

    start = time.time()
    last_flush = start
    rows = 0

    with open(OUTFILE, "w", newline="") as f:
        w = csv.writer(f)

        while True:
            line = ser.readline().decode(errors="replace").strip()
            if not line:
                continue

            if line == "DONE":
                print("Finished sampling.")
                break

            parts = line.split(",")
            if len(parts) != 8:
                continue
            
            #print header
            if parts[0] == "index":
                w.writerow(parts)
                f.flush()
                continue

            w.writerow(parts)
            rows += 1

            now = time.time()
            if now - last_flush > 10:
                f.flush()
                last_flush = now
                elapsed = now - start
                print(f"\rrows={rows} elapsed={elapsed/60:.1f} min", end="")

    print(f"Saved: {OUTFILE} ({rows} data rows)")


if __name__ == "__main__":
    main()