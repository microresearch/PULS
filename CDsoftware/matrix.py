# generate matrix example:
# but how to automate and also show which functions use 2x params...

fullmlists=[]

fullflist=[]
fullflistnames=[]


speedfunclist=["strobe", "spdfrac2", "spdfrac3", "spdfrac", "holdlspdfrac", "strobe", "ztogglebits", "ones", "clksr", "clksrG", "speedselcvl", "speedselcvm", "speedseloldcvl", "speedseloldcvm"]

lengthfunclist=["nlen", "rlen", "holdlen"]

adcfunclist=["zeros", "zadcx", "zadconebitsx", "zadconebitsxreset", "zadcpadbits", "zadc12bits", "zadc8bits", "zadc4bits", "zadceqbits", "zadcenergybits", "zadc12compbits", "zadc8compbits", "zadc4compbits", "zadccompbits", "zadc12onecompbits", "zadc8onecompbits", "zadc4onecompbits", "zadconecompbits", "adcselcvm", "adcselcvl", "probcvladcselcvm", "probdacadcsel"]

bitfunclist=["zeros", "binrout", "binroutfixed", "binroutor", "zsingleroutebits", "zbinrouteINVbits", "zbinroutebits_noshift_transit", "zbinroutebits_noshift", "zbinroutebitscycle", "zbinroutebitscyclestr", "zbinroutebitscycle_noshift", "zbinroutebitscyclestr_noshift", "zbinrouteORbits", "zbinrouteANDbits", "zbinrouteSRbits", "zbinroutebitsI", "zbinroutebitsI_noshift", "zbinroutebitscycleI_noshift", "zbinroutebitscyclestrI", "zosc1bits", "sigmadelta", "cipher", "osceq", "zSRclksr", "zSRclksrG", "zSRNbits", "zSRLbits", "zSRCbits", "zSRRbits", "zpulsebits", "zprobbits", "zprobbitsxorstrobe", "zprobbitsxortoggle", "zsuccbits", "zsuccbitsI", "zreturnbits", "zreturnnotbits", "zosc1bits", "zwiardbits", "zwiardinvbits", "zTMsimplebits", "zonebits", "zlfsrbits", "zllfsrbits", "zflipbits", "zosceqbitsI", "zosc1bitsI", "zTMsimplebitsI", "zwiardbitsI", "zwiardinvbitsI", "zonebitsI", "zlfsrbitsI", "zllfsrbitsI", "zflipbitsI", "zpattern4bits", "zpattern8bits", "zpattern4bitsI", "zpattern8bitsI", "Rtest", "gensel", "binroutfixed_prob1R", "binroutfixed_prob1L", "binroutfixed_prob2", "binroutfixed_prob3", "binroutfixed_prob4", "SRdelay_lineOUT"]

dacfunclist=["ddac0", "ddac1", "ddac2", "ddac3", "ddac4", "ddac5", "ddac6", "ddac7", "ddac8", "ddac9", "ddac10", "ddac11", "ddac12", "ddac13", "ddac14", "ddac15", "ddac16", "ddac17", "ddac18", "ddac19", "ddac20", "ddac21", "ddac22", "ddac23", "ddac24", "dacselcvl", "dacselcvm"]

newfunclist=["zero", "bitsmod", "cvmod"]

outfunclist=["vgen", "vxor"]

gsfunclist=["gshift0", "gshiftnull"]

foutfunclist=["OUT_zero", "OUT_SRdelay_lineIN"]

# func matrix:enum refs {fspeed, flength, fadc, fbit, fdac, fnew, fout, gs, out}; // 9

listoflists=[speedfunclist, lengthfunclist,adcfunclist,bitfunclist,dacfunclist,newfunclist,foutfunclist,gsfunclist,outfunclist]
names=["speed","length","adc","bit","dac","new", "fout", "gs","out"]
xx=0
for listy in listoflists:
    num=0
    print names[xx]+" :",
    for func in listy:
        print str(num)+":"+func, 
        num+=1
    print
    print
    x=input("select function?")
    print listy[x]
    fullflist.append(x)
    fullflistnames.append(listy[x])
    print
    xx+=1
    
# print as matrix but {0,0...}
fullmlists.append("{"+str(fullflist).strip("[]")+"}, //"+ str(fullflistnames))

# do cv - how to signal funcs which need dual CVs?
# cv matrix: enum cvs {cvspeed, cvspeedmod, cvlength, cvdac, cvadc, cvadcIN,  cvbit, cvbitcomp, cvnew, cvout, cvoutmod}; //11

# out is outside function
cvs=["cvspeed", "cvspeedmod", "cvlength", "cvdac", "cvadc", "cvadcIN", "cvbit", "cvbitcomp", "cvnew", "cvout"]
cvmatrix=["null", "0dac", "1dac", "2dac", "3dac", "CV", "CVL", "CVM", "ADCin", "Gs0", "Gs1", "Gs2", "Gs3", "clksr_", "param", "par", "oldcv", "oldcvl", "oldcvm"]

fullcvlist=[]
fullcvlistnames=[]
fullcvlists=[]

x=0
for cv in cvs:
    print str(cv)+"_:",
    xx=0
    for m in cvmatrix:
        print str(xx)+" "+m,
        xx+=1
    print
    mm=input("select "+ "cv?")
    print
    x+=1
    fullcvlist.append(mm)
    fullcvlistnames.append(cvmatrix[mm])

fullcvlists.append("{"+str(fullcvlist).strip("[]")+"}, //"+ str(fullcvlistnames))
    
#ending list fullmlists and then cvs
for listy in fullmlists:
    print listy

print
    
for listy in fullcvlists:
    print listy
    
