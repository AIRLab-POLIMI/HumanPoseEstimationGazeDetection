labels = []
file1 = open("labels.txt", 'r')
while True:
	line = file1.readline().rstrip().split()
	print(line)
	if len(line) == 1: line = " "
	elif len(line) == 2: line = line[1]
	elif len(line) == 3: line = line[1] + " " + line[2]
	print(line)
	labels.append(line)
	if not line:
		labels.pop()
		break
file1.close()
print(labels)
			
