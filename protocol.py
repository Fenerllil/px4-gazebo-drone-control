import struct


class CustomProtocol:
    msg_id = 0x01
    payload_len = 28

    @staticmethod
    def pack_pose(x,y,z,qw,qx,qy,qz):
        payload = struct.pack('<7f',x,y,z,qw,qx,qy,qz)
        # Считаем контрольную сумму
        crc = CustomProtocol.msg_id ^ CustomProtocol.payload_len
        for b in payload: crc ^= b
        return (b'\xAA' + bytes([CustomProtocol.msg_id, CustomProtocol.payload_len]) + payload + bytes([crc]))
    
    @staticmethod
    def unpack_pose(payload:bytes):
        return struct.unpack('<7f',payload)

    @classmethod
    def validate_crc(cls, msg_id:int,payload_len:int,payload:bytes, received_crc: int):
        calculated_crc = msg_id ^ payload_len
        for b in payload: calculated_crc ^= b
        return calculated_crc == received_crc