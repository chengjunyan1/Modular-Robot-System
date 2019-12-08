# Server Tools

def list2str(a):  
    s='['
    for i in a:
        s+=str(i)
        s+=','
    s=s[:-1]+']'
    return s

def str2list(s):
    return eval(s)