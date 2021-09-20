from six import b
import ROOT
import numpy as np
import pandas

df = ROOT.RDataFrame('myTree', '1_150.out')
t=ROOT.TTree('1_150', '1_150')
branches = "x/D:y/D"
t.ReadFile('1_150.out', branches)

N=t.GetEntries()
t.Show(200)
t.Print()

# np_array = df.Filter().Define('z', 'x*y').AsNumpy()
# pdf = pandas.DataFrame(np_array)
t.Draw("y:x", "x>=0.5 && x<=1.5", "*")
pass