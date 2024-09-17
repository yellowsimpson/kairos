left_gloves = [2, 1, 2, 2, 4]
right_gloves = [1, 2, 2, 4, 4, 7]

print(left_gloves, right_gloves)

left_dict={}
for i in left_gloves:
  if i in left_dict:
    left_dict[i]+=1
  else:
    left_dict[i]=1
print(left_dict)

right_dict={}
for i in right_gloves:
  if i in right_dict:
    right_dict[i]+=1
  else:
    right_dict[i]=1
print(right_dict)

total_dict={}
for i in right_gloves:
  if i in left_dict:
    A=min(right_dict[i],left_dict[i])
    total_dict[i]=A
  else:
    total_dict[i]=0
print(total_dict)

  
for i,j in enumerate(total_dict):
  print(f"size{j}는 {total_dict[j]}쌍입니다")