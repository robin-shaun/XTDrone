

def pm(omega,omega_p):
    ''' 
    Return spectrum value as a function of freq and peak freq
    '''
    alpha = 0.0081
    beta = 0.74
    g = 9.81
    return alpha*g**2/(omega**5)*exp(-(5.0/4.0)*(omega_p/omega)**4)




figure(1)
clf()
Tps = [pi]#2,4,6]
for Tp in Tps:
    omega_p = 2*pi/Tp
    omegas = linspace(omega_p*.25,omega_p*4,100)
    s = []
    for o in omegas:
        s.append(pm(o,omega_p))
    plot(omegas,s, label='$\\bar{T}=T_p$ = %.2f s, $\omega_p$=%.2f rad/s'%(Tp,omega_p))

    scale = 1.5
    omegas = [omega_p/scale, omega_p, omega_p*scale]
    delos = [omega_p*(1-1/scale), omega_p*(scale-1/scale)/2.0, omega_p*(scale-1)]
    s = []
    a = []
    for o,d in zip(omegas,delos):
        s.append(pm(o,omega_p))
        a.append(sqrt(2*pm(o,omega_p)*d))
    print (2*pi/omega_p)
    print (2*pi/array(omegas))
    print (a)     
    plot(omegas,s,'o',label='Samples for s = %.2f'%scale)

legend()    
grid(True)
xlabel('$\omega \,\, [rad/s]$')
ylabel('$S(\omega) \,\, [m^2/(rad/s)]$')
title('P-M Spectrum')
             
show()
