import struct
import csv
import os
import sys

INPUT_FILE = "IMULOG00.BIN"
OUTPUT_FILE = "DAQ_DECODED_DATA.csv"

# --- HARDWARE SCALING CONSTANTS ---
GRAVITY = 9.80665              # Standard gravity (m/s^2)

# ADXL359: 20-bit raw counts, +/- 40g range -> 78 ug/LSB
ADXL_SCALE = 0.000078          # g/LSB

# Murata SCH16T-K10: Standard sensitivities
SCH_RATE_SCALE = 200.0         # 200 LSB / (deg/s)
SCH_ACC_SCALE = 3200.0         # 3200 LSB / g

# Kionix KX134-1211: +/- 64g range (16-bit) -> 512 LSB/g
KX_ACC_SCALE = 512.0           # 512 LSB / g

# BNO086 Sensor IDs
SH2_ACCEL = 0x01
SH2_GYRO = 0x02
SH2_MAG = 0x03

def decode_bin(input_path=INPUT_FILE, output_path=OUTPUT_FILE):
    if not os.path.exists(input_path):
        print(f"Error: {input_path} not found!")
        return

    print(f"Decoding binary log: {input_path} -> {output_path}...")

    # Struct unpackers
    sch_unpacker  = struct.Struct("<QIhhhhhh")  # timestamp(8), dt(4), rx, ry, rz, ax, ay, az(6*2) = 24B
    adxl_hdr_fmt  = struct.Struct("<QIB")       # timestamp(8), dt(4), count(1) = 13B
    adxl_smp_fmt  = struct.Struct("<iii")       # ax(4), ay(4), az(4) = 12B
    kx_unpacker   = struct.Struct("<QIhhh")     # timestamp(8), dt(4), ax, ay, az(3*2) = 18B
    bno_unpacker  = struct.Struct("<QIBfff")    # timestamp(8), dt(4), sensorId(1), x, y, z(3*4) = 25B

    # Legacy ADXL single-sample unpacker for backward compatibility with v4 logs
    adxl_legacy_unpacker = struct.Struct("<QIiii") # 24B

    total_packets = 0
    sch_count = 0
    adxl_count = 0
    kx_count = 0
    bno_count = 0

    with open(input_path, "rb") as f_in, open(output_path, "w", newline='') as f_out:
        writer = csv.writer(f_out)

        # Unified telemetry CSV header
        writer.writerow([
            "Phase", "Sensor_ID", "Time_uS", "Delta_uS",
            "SCH_Deg_s_X", "SCH_Deg_s_Y", "SCH_Deg_s_Z",
            "SCH_m_s2_X", "SCH_m_s2_Y", "SCH_m_s2_Z",
            "ADXL_m_s2_X", "ADXL_m_s2_Y", "ADXL_m_s2_Z",
            "KX_m_s2_X", "KX_m_s2_Y", "KX_m_s2_Z",
            "BNO_m_s2_X", "BNO_m_s2_Y", "BNO_m_s2_Z",
            "BNO_Deg_s_X", "BNO_Deg_s_Y", "BNO_Deg_s_Z",
            "BNO_Mag_uT_X", "BNO_Mag_uT_Y", "BNO_Mag_uT_Z"
        ])

        while True:
            byte1 = f_in.read(1)
            if not byte1:
                break

            if byte1 == b'\xAA':
                byte2 = f_in.read(1)
                if byte2 == b'\xBB':
                    p_type = f_in.read(1)
                    phase_byte = f_in.read(1)
                    if not phase_byte:
                        break
                    phase = int.from_bytes(phase_byte, byteorder='little')

                    # -------------------------------------------------------------
                    # 1. Murata SCH16T-K10 Parsing ('S')
                    # -------------------------------------------------------------
                    if p_type == b'S':
                        payload = f_in.read(24)
                        if len(payload) < 24:
                            break
                        t_stamp, dt, rx, ry, rz, ax, ay, az = sch_unpacker.unpack(payload)

                        gX = rx / SCH_RATE_SCALE
                        gY = ry / SCH_RATE_SCALE
                        gZ = rz / SCH_RATE_SCALE

                        aX = (ax / SCH_ACC_SCALE) * GRAVITY
                        aY = (ay / SCH_ACC_SCALE) * GRAVITY
                        aZ = (az / SCH_ACC_SCALE) * GRAVITY

                        writer.writerow([
                            phase, "S", t_stamp, dt,
                            f"{gX:.3f}", f"{gY:.3f}", f"{gZ:.3f}",
                            f"{aX:.3f}", f"{aY:.3f}", f"{aZ:.3f}",
                            "", "", "",
                            "", "", "",
                            "", "", "",
                            "", "", "",
                            "", "", ""
                        ])
                        sch_count += 1
                        total_packets += 1

                    # -------------------------------------------------------------
                    # 2. ADXL359 Batched or Legacy Parsing ('A')
                    # -------------------------------------------------------------
                    elif p_type == b'A':
                        # Peek next bytes to distinguish between Batch (13B header) and Legacy (24B)
                        hdr_data = f_in.read(13)
                        if len(hdr_data) < 13:
                            break

                        t_batch_end, dt_sample, count = adxl_hdr_fmt.unpack(hdr_data)

                        if count == 30:
                            # New Batched Format
                            samples_data = f_in.read(30 * 12)
                            if len(samples_data) < 360:
                                break

                            for i in range(30):
                                chunk = samples_data[i*12 : (i+1)*12]
                                raw_x, raw_y, raw_z = adxl_smp_fmt.unpack(chunk)

                                # Timeline reconstruction: batch ends at t_batch_end
                                sample_time = t_batch_end - ((29 - i) * dt_sample)

                                aX = (raw_x * ADXL_SCALE) * GRAVITY
                                aY = (raw_y * ADXL_SCALE) * GRAVITY
                                aZ = (raw_z * ADXL_SCALE) * GRAVITY

                                writer.writerow([
                                    phase, "A", sample_time, dt_sample,
                                    "", "", "", "", "", "",
                                    f"{aX:.3f}", f"{aY:.3f}", f"{aZ:.3f}",
                                    "", "", "",
                                    "", "", "",
                                    "", "", "",
                                    "", "", ""
                                ])
                                adxl_count += 1
                                total_packets += 1
                        else:
                            # Fallback / Legacy single sample packet
                            f_in.seek(-13, os.SEEK_CUR)
                            payload = f_in.read(24)
                            if len(payload) < 24:
                                break
                            t_stamp, dt, raw_x, raw_y, raw_z = adxl_legacy_unpacker.unpack(payload)

                            aX = (raw_x * ADXL_SCALE) * GRAVITY
                            aY = (raw_y * ADXL_SCALE) * GRAVITY
                            aZ = (raw_z * ADXL_SCALE) * GRAVITY

                            writer.writerow([
                                phase, "A", t_stamp, dt,
                                "", "", "", "", "", "",
                                f"{aX:.3f}", f"{aY:.3f}", f"{aZ:.3f}",
                                "", "", "",
                                "", "", "",
                                "", "", "",
                                "", "", ""
                            ])
                            adxl_count += 1
                            total_packets += 1

                    # -------------------------------------------------------------
                    # 3. Kionix KX134-1211 High-G Parsing ('K')
                    # -------------------------------------------------------------
                    elif p_type == b'K':
                        payload = f_in.read(18)
                        if len(payload) < 18:
                            break
                        t_stamp, dt, raw_x, raw_y, raw_z = kx_unpacker.unpack(payload)

                        aX = (raw_x / KX_ACC_SCALE) * GRAVITY
                        aY = (raw_y / KX_ACC_SCALE) * GRAVITY
                        aZ = (raw_z / KX_ACC_SCALE) * GRAVITY

                        writer.writerow([
                            phase, "K", t_stamp, dt,
                            "", "", "", "", "", "",
                            "", "", "",
                            f"{aX:.3f}", f"{aY:.3f}", f"{aZ:.3f}",
                            "", "", "",
                            "", "", "",
                            "", "", ""
                        ])
                        kx_count += 1
                        total_packets += 1

                    # -------------------------------------------------------------
                    # 4. CEVA / Hillcrest BNO086 Parsing ('B')
                    # -------------------------------------------------------------
                    elif p_type == b'B':
                        payload = f_in.read(25)
                        if len(payload) < 25:
                            break
                        t_stamp, dt, s_id, x, y, z = bno_unpacker.unpack(payload)

                        row = [phase, "B", t_stamp, dt, "", "", "", "", "", "", "", "", "", "", "", ""]

                        if s_id == SH2_ACCEL:
                            row.extend([f"{x:.3f}", f"{y:.3f}", f"{z:.3f}", "", "", "", "", "", ""])
                        elif s_id == SH2_GYRO:
                            xDeg = x * 57.2957795
                            yDeg = y * 57.2957795
                            zDeg = z * 57.2957795
                            row.extend(["", "", "", f"{xDeg:.3f}", f"{yDeg:.3f}", f"{zDeg:.3f}", "", "", ""])
                        elif s_id == SH2_MAG:
                            row.extend(["", "", "", "", "", "", f"{x:.3f}", f"{y:.3f}", f"{z:.3f}"])
                        else:
                            continue

                        writer.writerow(row)
                        bno_count += 1
                        total_packets += 1

    print("\n--- DECODING SUMMARY ---")
    print(f"Total Decoded Frames : {total_packets}")
    print(f"  SCH16T Samples     : {sch_count}")
    print(f"  ADXL359 Samples    : {adxl_count}")
    print(f"  KX134 Samples      : {kx_count}")
    print(f"  BNO086 Samples     : {bno_count}")
    print(f"Successfully saved to: {output_path}")

if __name__ == "__main__":
    in_file = sys.argv[1] if len(sys.argv) > 1 else INPUT_FILE
    out_file = sys.argv[2] if len(sys.argv) > 2 else OUTPUT_FILE
    decode_bin(in_file, out_file)