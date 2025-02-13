import struct

def bytes_to_fp32(bytes_data, is_big_endian=False):
    return struct.unpack('>f' if is_big_endian else '<f', bytes_data)[0]


def bytes_to_fp32_list(bytes_data, n=0, is_big_endian=False):
    ret = []
    count = n if n > 0 else len(bytes_data) // 4
    for i in range(count):
        ret.append(bytes_to_fp32(bytes_data[i * 4: i * 4 + 4], is_big_endian))
    return ret

def bytes_to_u32(data, is_big_endian=False):
    return struct.unpack('>I' if is_big_endian else '<I', data)[0]



def bytes_to_u32_list(bytes_data, n=0, is_big_endian=False):
    ret = []
    count = n if n > 0 else len(bytes_data) // 4
    for i in range(count):
        ret.append(bytes_to_u32(bytes_data[i * 4: i * 4 + 4], is_big_endian))
    return ret

def bytes_to_u8(data):
    return data[0]

def bytes_to_u8_list(bytes_data):
    return list(bytes_data)

def bytes_to_int8(data):
    return struct.unpack('b', data)[0]

def bytes_to_int8_list(bytes_data):
    ret = []
    for i in range(len(bytes_data)):
        ret.append(bytes_to_int8(bytes_data[i:i + 1]))
    return ret

def bytes_to_int32(bytes_data, is_big_endian=False):
    return struct.unpack('>i' if is_big_endian else '<i', bytes_data)[0]

def bytes_to_int32_list(bytes_data, n=0, is_big_endian=False):
    ret = []
    count = n if n > 0 else len(bytes_data) // 4
    for i in range(count):
        ret.append(bytes_to_int32(bytes_data[i * 4: i * 4 + 4], is_big_endian))
    return ret

def bytes_to_int16(bytes_data, is_big_endian=False):
    return struct.unpack('>h' if is_big_endian else '<h', bytes_data)[0]

def bytes_to_int16_list(bytes_data, n=0, is_big_endian=False):
    ret = []
    count = n if n > 0 else len(bytes_data) // 2
    for i in range(count):
        ret.append(bytes_to_int16(bytes_data[i * 2: i * 2 + 2], is_big_endian))
    return ret

def bytes_to_u16(bytes_data, is_big_endian=False):
    return struct.unpack('>H' if is_big_endian else '<H', bytes_data)[0]

def bytes_to_u16_list(bytes_data, n=0, is_big_endian=False):
    ret = []
    count = n if n > 0 else len(bytes_data) // 2
    for i in range(count):
        ret.append(bytes_to_u16(bytes_data[i * 2: i * 2 + 2], is_big_endian))
    return ret



if __name__ == '__main__':
   # Example usage
    bytes_data = b'\x01\x02\x03\x04\x05\x06\x07\x08'
    print(bytes_to_u8(bytes_data[0:1]))  # 1
    print(bytes_to_u8_list(bytes_data))  # [1, 2, 3, 4, 5, 6, 7, 8]
    print(bytes_to_int8(bytes_data[0:1]))  # 1
    print(bytes_to_int8_list(bytes_data))  # [1, 2, 3, 4, 5, 6, 7, 8]
    print(bytes_to_int32(bytes_data[0:4]))  # 67305985
    print(bytes_to_int32_list(bytes_data))  # [67305985, 134678021, 202050057]
    print(bytes_to_int16(bytes_data[0:2]))  # 513
    print(bytes_to_int16_list(bytes_data))  # [513, 1027, 1541, 2055]
    print(bytes_to_u16(bytes_data[0:2]))  # 513
    print(bytes_to_u16_list(bytes_data))  # [513, 1027, 1541, 2055]
    print(bytes_to_u32(bytes_data[0:4]))  
    print(bytes_to_u32_list(bytes_data[0:4]))  