/**
 * pcap-writer.js — Minimal libpcap (DLT_EN10MB) writer for UDP payloads.
 *
 * Each call to write(payload) frames the payload as a fake Ethernet+IPv4+UDP
 * packet so standard tools (Wireshark, Ouster SDK) can read it back.
 */

import fs from 'fs';

const MAGIC = 0xa1b2c3d4;
const DLT_EN10MB = 1;

function ipChecksum(buf, start, end) {
  let sum = 0;
  for (let i = start; i < end; i += 2) sum += buf.readUInt16BE(i);
  while (sum >> 16) sum = (sum & 0xffff) + (sum >> 16);
  return (~sum) & 0xffff;
}

export class PcapWriter {
  constructor(filepath, { srcIp = '192.168.1.210', dstIp = '192.168.1.1', srcMac, dstMac } = {}) {
    this.filepath = filepath;
    this.srcIp = srcIp.split('.').map(Number);
    this.dstIp = dstIp.split('.').map(Number);
    this.srcMac = srcMac || [0xbc, 0x0f, 0xa7, 0x00, 0x17, 0x02];
    this.dstMac = dstMac || [0xc8, 0x4d, 0x44, 0x26, 0x3b, 0xa6];
    this.packetCount = 0;
    this.byteCount = 0;

    this.fd = fs.openSync(filepath, 'w');
    const hdr = Buffer.alloc(24);
    hdr.writeUInt32LE(MAGIC, 0);
    hdr.writeUInt16LE(2, 4);        // version_major
    hdr.writeUInt16LE(4, 6);        // version_minor
    hdr.writeInt32LE(0, 8);         // thiszone
    hdr.writeUInt32LE(0, 12);       // sigfigs
    hdr.writeUInt32LE(65535, 16);   // snaplen
    hdr.writeUInt32LE(DLT_EN10MB, 20);
    fs.writeSync(this.fd, hdr);
  }

  write(payload, { srcPort = 40000, dstPort = 7502 } = {}) {
    if (this.fd === null) return;
    const ethLen = 14;
    const ipLen = 20;
    const udpLen = 8;
    const total = ethLen + ipLen + udpLen + payload.length;

    const frame = Buffer.alloc(total);
    // Ethernet
    for (let i = 0; i < 6; i++) frame[i] = this.dstMac[i];
    for (let i = 0; i < 6; i++) frame[6 + i] = this.srcMac[i];
    frame.writeUInt16BE(0x0800, 12); // IPv4
    // IPv4
    const ipOff = ethLen;
    frame[ipOff] = 0x45;                                    // ver=4, ihl=5
    frame[ipOff + 1] = 0;                                   // DSCP/ECN
    frame.writeUInt16BE(ipLen + udpLen + payload.length, ipOff + 2); // total length
    frame.writeUInt16BE(this.packetCount & 0xffff, ipOff + 4);       // id
    frame.writeUInt16BE(0x4000, ipOff + 6);                 // flags (DF)
    frame[ipOff + 8] = 64;                                  // TTL
    frame[ipOff + 9] = 17;                                  // UDP
    frame.writeUInt16BE(0, ipOff + 10);                     // checksum placeholder
    for (let i = 0; i < 4; i++) frame[ipOff + 12 + i] = this.srcIp[i];
    for (let i = 0; i < 4; i++) frame[ipOff + 16 + i] = this.dstIp[i];
    const ipCk = ipChecksum(frame, ipOff, ipOff + ipLen);
    frame.writeUInt16BE(ipCk, ipOff + 10);
    // UDP
    const udpOff = ipOff + ipLen;
    frame.writeUInt16BE(srcPort, udpOff);
    frame.writeUInt16BE(dstPort, udpOff + 2);
    frame.writeUInt16BE(udpLen + payload.length, udpOff + 4);
    frame.writeUInt16BE(0, udpOff + 6); // checksum zero (optional for IPv4)
    // Payload
    payload.copy(frame, udpOff + udpLen);

    // PCAP record header
    const now = Date.now();
    const rec = Buffer.alloc(16);
    rec.writeUInt32LE(Math.floor(now / 1000), 0);
    rec.writeUInt32LE((now % 1000) * 1000, 4);
    rec.writeUInt32LE(total, 8);
    rec.writeUInt32LE(total, 12);
    fs.writeSync(this.fd, rec);
    fs.writeSync(this.fd, frame);

    this.packetCount++;
    this.byteCount += total;
  }

  close() {
    if (this.fd !== null) {
      fs.closeSync(this.fd);
      this.fd = null;
    }
    return { packets: this.packetCount, bytes: this.byteCount };
  }
}
