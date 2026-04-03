import struct
import csv
import os

INPUT_FILE = "IMULOG0009.BIN"
OUTPUT_FILE = "DAQ_DECODED_0009.csv"

# --- HARDWARE SCALING ALGORITHMS ---
ADXL_SCALE = 0.000078  
SCH_ACC_SCALE = 400.0   # Corrected for the 0x12DB register state
SCH_RATE_SCALE = 6.25   

GRAVITY = 9.80665       # Standard gravity for unit conversion

SH2_ACCEL = 0x01
SH2_GYRO = 0x02
SH2_MAG = 0x03

def decode_bin():
    if not os.path.exists(INPUT_FILE):
        print(f"Error: {INPUT_FILE} not found!")
        return

    print(f"Decoding {INPUT_FILE}...")

    # Struct unpackers 
    sch_unpacker = struct.Struct("<QIhhhhhh")  
    adxl_unpacker = struct.Struct("<QIiii")    
    bno_unpacker = struct.Struct("<QIBfff")    

    with open(INPUT_FILE, "rb") as f_in, open(OUTPUT_FILE, "w", newline='') as f_out:
        writer = csv.writer(f_out)
        
        # Headers updated to explicitly show everything is in m/s^2 and Deg/s
        writer.writerow([
            "Phase", "ID", "Time_uS", "Delta_uS", 
            "SCH_Deg_s_X", "SCH_Deg_s_Y", "SCH_Deg_s_Z", "SCH_m_s2_X", "SCH_m_s2_Y", "SCH_m_s2_Z", 
            "ADXL_m_s2_X", "ADXL_m_s2_Y", "ADXL_m_s2_Z",
            "BNO_m_s2_X", "BNO_m_s2_Y", "BNO_m_s2_Z",
            "BNO_Deg_s_X", "BNO_Deg_s_Y", "BNO_Deg_s_Z",
            "BNO_Mag_X", "BNO_Mag_Y", "BNO_Mag_Z"
        ])

        packets = 0

        while True:
            byte1 = f_in.read(1)
            if not byte1: break
            
            if byte1 == b'\xAA':
                byte2 = f_in.read(1)
                if byte2 == b'\xBB':
                    p_type = f_in.read(1)
                    
                    # 1. READ THE FLIGHT PHASE
                    phase_byte = f_in.read(1)
                    if not phase_byte: break
                    phase = int.from_bytes(phase_byte, byteorder='little')
                    
                    # --- 2. SCH16T PARSING ---
                    if p_type == b'S':
                        payload = f_in.read(24)
                        if len(payload) < 24: break
                        t_stamp, dt, rx, ry, rz, ax, ay, az = sch_unpacker.unpack(payload)
                        
                        gX = rx / SCH_RATE_SCALE
                        gY = ry / SCH_RATE_SCALE
                        gZ = rz / SCH_RATE_SCALE
                        
                        # Native m/s^2 output
                        aX = ax / SCH_ACC_SCALE
                        aY = ay / SCH_ACC_SCALE
                        aZ = az / SCH_ACC_SCALE
                        
                        writer.writerow([phase, "S", t_stamp, dt, 
                                         f"{gX:.3f}", f"{gY:.3f}", f"{gZ:.3f}", 
                                         f"{aX:.3f}", f"{aY:.3f}", f"{aZ:.3f}",
                                         "", "", "", "", "", "", "", "", "", "", "", ""])
                        packets += 1

                    # --- 3. ADXL359 PARSING ---
                    elif p_type == b'A':
                        payload = f_in.read(24)
                        if len(payload) < 24: break
                        t_stamp, dt, ax, ay, az = adxl_unpacker.unpack(payload)
                        
                        # Convert ADXL to m/s^2
                        aX = (ax * ADXL_SCALE) * GRAVITY
                        aY = (ay * ADXL_SCALE) * GRAVITY
                        
                        # INVERT Z-AXIS to match the frame
                        aZ = -(az * ADXL_SCALE) * GRAVITY
                        
                        writer.writerow([phase, "A", t_stamp, dt, 
                                         "", "", "", "", "", "", 
                                         f"{aX:.3f}", f"{aY:.3f}", f"{aZ:.3f}",
                                         "", "", "", "", "", "", "", "", ""])
                        packets += 1

                    # --- 4. BNO086 PARSING ---
                    elif p_type == b'B':
                        payload = f_in.read(25)
                        if len(payload) < 25: break
                        t_stamp, dt, s_id, x, y, z = bno_unpacker.unpack(payload)
                        
                        row = [phase, "B", t_stamp, dt, "", "", "", "", "", "", "", "", ""]
                        
                        if s_id == SH2_ACCEL:
                            # BNO natively outputs in m/s^2
                            row.extend([f"{x:.3f}", f"{y:.3f}", f"{z:.3f}", "", "", "", "", "", ""])
                        
                        elif s_id == SH2_GYRO:
                            # BNO outputs Rad/s. Convert to Deg/s
                            xDeg = x * 57.2958
                            yDeg = y * 57.2958
                            zDeg = z * 57.2958
                            row.extend(["", "", "", f"{xDeg:.3f}", f"{yDeg:.3f}", f"{zDeg:.3f}", "", "", ""])
                            
                        elif s_id == SH2_MAG:
                            row.extend(["", "", "", "", "", "", f"{x:.3f}", f"{y:.3f}", f"{z:.3f}"])
                        else:
                            continue 
                        
                        writer.writerow(row)
                        packets += 1

    print(f"Decoding complete! Parsed {packets} total frames.")
    print(f"Saved to {OUTPUT_FILE}")

if __name__ == "__main__":
    decode_bin()