count=0
text = open('main.c', 'r').read()
l = text.split('\n')

non_comment_lines = []

for line in l:
    if not line.strip().startswith("//") and not line.strip().startswith("/*"):
        non_comment_lines.append(line)

print("\n")
print("LOC: ", len(l))
print("CLOC no blank: ", len([i for i in filter(lambda x: x != "", non_comment_lines)]))
print("NLOC: ", len([i for i in filter(lambda x: x!='', non_comment_lines)]))
print("\n")
