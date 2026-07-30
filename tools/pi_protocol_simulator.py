#!/usr/bin/env python3
import argparse, struct, time
SOF=b'\xA5\x5A'; VER=1

def crc16(data: bytes)->int:
    c=0xFFFF
    for x in data:
        c ^= x<<8
        for _ in range(8): c=((c<<1)^0x1021)&0xFFFF if c&0x8000 else (c<<1)&0xFFFF
    return c

def frame(msg_type:int, seq:int, ts:int, payload:bytes)->bytes:
    body=struct.pack('<BBHII',VER,msg_type,len(payload),seq,ts)+payload
    return SOF+body+struct.pack('<H',crc16(body))

def ball(seq:int,pos_cm:float,vel_cm_s:float,valid=True)->bytes:
    flags=0x01|(0x04 if valid else 0)
    payload=struct.pack('<IhhHHBBH',seq,round(pos_cm*100),round(vel_cm_s*100),950,20,flags,0,0)
    return frame(0x01,seq,int(time.monotonic()*1000)&0xFFFFFFFF,payload)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--port');ap.add_argument('--baud',type=int,default=115200);ap.add_argument('--loopback',action='store_true');ap.add_argument('--dry',action='store_true');args=ap.parse_args()
    if args.dry or not args.port:
        print(ball(1,0.0,0.0).hex(' '));return
    try: import serial
    except ImportError: raise SystemExit('Install pyserial: python -m pip install pyserial')
    with serial.Serial(args.port,args.baud,timeout=0.05) as s:
        seq=0
        while True:
            p=ball(seq,0.0,0.0);s.write(p);data=s.read(256)
            if data: print(data.hex(' '))
            seq+=1;time.sleep(0.025)
if __name__=='__main__': main()
