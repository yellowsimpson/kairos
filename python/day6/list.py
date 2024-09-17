left_gloves = [2, 1, 2, 2, 4]
right_gloves = [1, 2, 50, 2, 100]
size_dict = {}

left_gloves.sort()
right_gloves.sort()

for size in left_gloves:
    if size in size_dict:
        size_dict[size][0] += 1
    else:
        size_dict[size] = [1, 0]

print(size_dict)

pair_count = {size: min(counts) for size, counts in size_dict.items()}
print(pair_count)
