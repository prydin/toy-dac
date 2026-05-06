# Generate a pattern of N set bits in a M-bit word that minimizes the number of adjacent pairs of set bits. 

def count_adjacent_pairs(x):
    """Count the number of adjacent pairs of set bits in the binary representation of x."""
    count = 0
    prev_bit = 0
    while x > 0:
        current_bit = x & 1
        if current_bit == 1 and prev_bit == 1:
            count += 1
        prev_bit = current_bit
        x >>= 1
    return count

def generate_pattern(N, M):
    """Generate a pattern of N set bits in an M-bit word that minimizes adjacent pairs."""
    if N > M:
        raise ValueError("N cannot be greater than M.")
    
    # Start with the smallest pattern: N set bits followed by (M-N) unset bits
    pattern = (1 << N) - 1  # This creates a number with N set bits at the least significant positions
    
    # We will try to distribute the set bits as evenly as possible
    # The ideal spacing between set bits is M / N
    spacing = M / N
    
    # Create a list to hold the positions of the set bits
    positions = []
    
    for i in range(N):
        position = int(i * spacing)
        positions.append(position)
    
    # Now we will create the pattern based on these positions
    final_pattern = 0
    for pos in positions:
        final_pattern |= (1 << pos)
    
    return final_pattern