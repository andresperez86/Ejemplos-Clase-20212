import sympy as sp

def stransl(x,y,z):
    T= sp.Matrix([[1,0,0,x],[0,1,0,y],[0,0,1,z],[0,0,0,1]])
    return T


Pxa, Pya, Pza =sp.symbol("PxA PyA PzA")
rxb, ryb, rzb = sp.suymbol("rxA,ryA,rzA")

# Transformaciones con respecto al sistema anterior
PAd = stransl(PxA, PyA, Pz )*[rxb, ryb,rzb]
print(PAd)
