import crc8

hash = crc8.crc8()
hash.update(b'\xAA\x00\x81\x00\x00\x00\x04')
print(hash.hexdigest())