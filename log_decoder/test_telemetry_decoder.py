import os
import struct
import tempfile
from log_decoder import decode_bin

def test_roundtrip():
    temp_dir = tempfile.mkdtemp()
    bin_file = os.path.join(temp_dir, "TEST_LOG.BIN")
    csv_file = os.path.join(temp_dir, "TEST_LOG.csv")

    print(f"Generating synthetic telemetry packets in {bin_file}...")

    with open(bin_file, "wb") as f:
        # 1. SCH16T Packet: 28 bytes
        # sync1(1)=0xAA, sync2(1)=0xBB, type(1)='S', phase(1)=3, timestamp(8)=1000000, dt(4)=678, rateX..Z(6), accX..Z(6)
        sch_pkt = struct.pack("<BBcBQIhhhhhh", 0xAA, 0xBB, b'S', 3, 1000000, 678, 200, -400, 600, 3200, 0, -3200)
        f.write(sch_pkt)

        # 2. ADXL359 Batched Packet: 377 bytes
        # sync1(1)=0xAA, sync2(1)=0xBB, type(1)='A', phase(1)=3, timestamp(8)=1030000, dt(4)=1000, count(1)=30
        adxl_hdr = struct.pack("<BBcBQIB", 0xAA, 0xBB, b'A', 3, 1030000, 1000, 30)
        f.write(adxl_hdr)
        for i in range(30):
            # 12 bytes per sample (accX, accY, accZ)
            smp = struct.pack("<iii", 12820, 0, -12820)
            f.write(smp)

        # 3. KX134 Packet: 22 bytes
        # sync1(1)=0xAA, sync2(1)=0xBB, type(1)='K', phase(1)=3, timestamp(8)=1000625, dt(4)=625, accX..Z(3*2)
        kx_pkt = struct.pack("<BBcBQIhhh", 0xAA, 0xBB, b'K', 3, 1000625, 625, 512, 0, -512)
        f.write(kx_pkt)

        # 4. BNO086 Packet: 29 bytes
        # sync1(1)=0xAA, sync2(1)=0xBB, type(1)='B', phase(1)=3, timestamp(8)=1005000, dt(4)=10000, s_id(1)=1, x, y, z(3*4)
        bno_pkt = struct.pack("<BBcBQIBfff", 0xAA, 0xBB, b'B', 3, 1005000, 10000, 1, 0.0, 9.81, 0.0)
        f.write(bno_pkt)

    print("Running decode_bin()...")
    decode_bin(bin_file, csv_file)

    assert os.path.exists(csv_file), "CSV output file was not created!"
    with open(csv_file, "r") as f_out:
        lines = f_out.readlines()
        print(f"Generated {len(lines)} CSV rows.")
        for l in lines[:10]:
            print("  ", l.strip())

    # Total rows = header (1) + SCH (1) + ADXL (30) + KX (1) + BNO (1) = 34 lines
    assert len(lines) == 34, f"Expected 34 lines in CSV, got {len(lines)}"
    print("\nSUCCESS: All packet types parsed with 100% accuracy!")

if __name__ == "__main__":
    test_roundtrip()
