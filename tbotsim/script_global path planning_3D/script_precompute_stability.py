
from __future__     import annotations
import numpy as np
import tbotlib as tb
import os

if __name__ == "__main__":

    start = [[0]]
    stop = [[1000000]]
    step = [[1]]

    d = tb.ArangeDict.create(start, stop, step)
    
    tb.tic()
    
    for i in range(d.len()):
        d.assign([i], i)

    tb.toc()

    d.print_size()

    file = os.path.join(os.path.dirname(__file__), 'data/test.json')
    d.save(file)