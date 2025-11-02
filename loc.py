count=0
text = open('main.c', 'r').read()
l = text.split('\n')

non_comment_lines = []

pure_code = 0
no_count = False
for line in l:
    if "//" in line:
        continue

    if no_count:
        if "*/" in line:
            no_count = False
        continue
    else:
        if line.strip().startswith("/*"):
            no_count = True
            if line.strip().endswith("*/"):
                no_count = False
            continue

        pure_code += 1
    

print("LOC: ", len(l))
print("CLOC no blank: ", len([i for i in filter(lambda x: x != "", l)]))
print("C CODE: ", pure_code)
