import csv
import time
import serial

PORT = "COM6"
BAUD = 115200
OUTFILE = "allan_log.csv"
DURATION_S = 7 * 3600

bad = 0
last_debug = time.time()

def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    raw = ser.readline().decode(errors="replace").strip()
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
            if len(parts) != 14:
                print(f"Bad line: {line}")
                continue
            
            #print header
            if parts[0] == "index":
                print("Writing header")
                w.writerow(parts)
                f.flush()
                continue

            w.writerow(parts)
            print(f"Logged row: {parts[0]}")
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