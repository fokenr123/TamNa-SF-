import sys
from collections import Counter
n = int(sys.stdin.readline())
l = []
for _ in range(n):
    l.append(int(sys.stdin.readline()))
# 평균
print(round(sum(l)/len(l)))
# 중앙값
r = sorted(l)
print(r[len(r)//2])
# 최빈값
cnt = Counter(l)
cnt = cnt.most_common()
cnt = sorted(cnt, key = lambda x : (-x[1], x[0]))
if len(l) > 1:
    if cnt[0][1] == cnt[1][1]:
        print(cnt[1][0])
    else:
        print(cnt[0][0])
else:
    print(cnt[0][0])
# 범위
print(max(l)-min(l) )
