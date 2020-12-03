import itertools

def createHistogram(data, binCount, binBounds=None):
    els = list(itertools.repeat(0,binCount))
    if binBounds is None:
        binBounds =(min(data),max(data))
    binSize = (binBounds[1]-binBounds[0]) / binCount
    for el in data:
        pos = max(0,min(int((el-binBounds[0])/binSize), binCount-1))
        els[pos] += 1
    return (els,binBounds)
