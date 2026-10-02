"""Generate reciprocal paper-scale ReSTIR PT pairing maps."""

import struct
from pathlib import Path


MASK = 0xFFFFFFFF
SIZES = (254, 230, 210, 190, 178, 166, 154, 142, 246, 238, 222, 206, 198, 182, 174, 162)
SHUFFLE_ITERATIONS = 129  # Sigma approximately 16 pixels.


def hash32(value):
    value ^= value >> 16
    value = value * 0x7FEB352D & MASK
    value ^= value >> 15
    value = value * 0x846CA68B & MASK
    return (value ^ (value >> 16)) & MASK


def random_float(state):
    old = state
    state = (old * 747796405 + 2891336453) & MASK
    word = (((old >> ((old >> 28) + 4)) ^ old) * 277803737) & MASK
    value = ((word >> 22) ^ word) & MASK
    return state, (value >> 9) / (1 << 23)


def generate(dimension, iterations):
    links = [index // 2 for index in range(dimension * dimension)]
    for iteration in range(iterations):
        shuffled = [0] * len(links)
        for y in range(dimension // 2):
            for x in range(dimension // 2):
                start_x = (2 * x + iteration % 2) % dimension
                start_y = (2 * y + iteration % 2) % dimension
                coords = ((start_x, start_y), ((start_x + 1) % dimension, start_y),
                          (start_x, (start_y + 1) % dimension),
                          ((start_x + 1) % dimension, (start_y + 1) % dimension))
                indices = [py * dimension + px for px, py in coords]
                ids = [links[index] for index in indices]
                pixel = y * dimension + x
                seed = ((pixel >> 16) ^ pixel) * 0x45D9F3B & MASK
                seed = ((seed >> 16) ^ seed) * 0x45D9F3B & MASK
                seed = ((seed >> 16) ^ seed) & MASK
                depth = hash32(dimension)
                depth ^= depth >> 16
                depth = depth * 0x85EBCA6B & MASK
                depth ^= depth >> 13
                depth = depth * 0xC2B2AE35 & MASK
                depth = (depth ^ (depth >> 16)) & MASK
                state = (seed ^ (iteration * 1973 & MASK) ^ depth) | 1
                state, _ = random_float(state)
                for i in range(3, 0, -1):
                    state, value = random_float(state)
                    j = min(int(value * (i + 1)), i)
                    ids[i], ids[j] = ids[j], ids[i]
                for index, link in zip(indices, ids):
                    shuffled[index] = link
        links = shuffled

    positions = [[] for _ in range(len(links) // 2)]
    for index, link in enumerate(links):
        positions[link].append(index)
    assert all(len(pair) == 2 for pair in positions)
    deltas = [(0, 0)] * len(links)
    for first, second in positions:
        for source, target in ((first, second), (second, first)):
            dx = target % dimension - source % dimension
            dy = target // dimension - source // dimension
            if dx > dimension // 2:
                dx -= dimension
            if dx < -dimension // 2:
                dx += dimension
            if dy > dimension // 2:
                dy -= dimension
            if dy < -dimension // 2:
                dy += dimension
            deltas[source] = dx, dy
    for index, (dx, dy) in enumerate(deltas):
        assert dx or dy
        px = (index % dimension + dx) % dimension
        py = (index // dimension + dy) % dimension
        assert deltas[py * dimension + px] == (-dx, -dy)
    return [((dy & 0xFFFF) << 16) | (dx & 0xFFFF) for dx, dy in deltas]


def main():
    path = Path(__file__).resolve().parents[1] / "EvoEngine_SDK/Internals/DefaultResources/RestirPtPairing.bin"
    with path.open("wb") as file:
        for size in SIZES:
            values = generate(size, SHUFFLE_ITERATIONS)
            file.write(struct.pack(f"<{len(values)}I", *values))
    print(f"{path}: {path.stat().st_size} bytes")


if __name__ == "__main__":
    main()
