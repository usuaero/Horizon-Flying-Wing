
def boundActuators(sym, asym, d=20.):
    s = list(sym[1:])
    a = asym[:]
    for i in range(5):
        r = s[i] + a[i]
        l = s[i] - a[i]
        flag = False
        if r >  d:
            r =  d
            flag = True
        elif r < -d:
            r = -d
            flag = True
        if l >  d:
            l =  d
            flag = True
        elif l < -d:
            l = -d
            flag = True
        if flag:
            s[i] = (r+l)/2.
            a[i] = (r-l)/2.
    return [sym[0]]+s, a


def mode1(dl, dm, d=20.):
    return boundActuators([d*dm]*6, [d*dl]*5, d=d)

def mode2(dL, dl, dm):
    
    CL1 = dL * 0.9
    Cm1 = dm * 0.1
    pbar1 = dl * 0.2
    
    CL0 = 1.
    CL2 = CL1 * CL1
    CL3 = CL2 * CL1
    
    Cm0 = 1.
    Cm2 = Cm1 * Cm1
    Cm3 = Cm2 * Cm1
    Cm4 = Cm3 * Cm1
    Cm5 = Cm4 * Cm1
    
    pbar0 = 1.
    pbar2 = pbar1 * pbar1
    pbar3 = pbar2 * pbar1
    pbar4 = pbar3 * pbar1
    pbar5 = pbar4 * pbar1
    pbar6 = pbar5 * pbar1
    
    sym  = [0.]*6
    asym = [0.]*5
    
    sym[0] = CL0 * (Cm0 * (-3.5911697905177062 * pbar0 + -17.76881662698185 * pbar2 + 740.0743524877553 * pbar4 + -7369.391708839217 * pbar6)  + Cm1 * (-86.59430963196279 * pbar0 + 351.4052689845445 * pbar2 + 3191.404008456966 * pbar4 + -50289.67792846789 * pbar6)  + Cm2 * (63.53047717606253 * pbar0 + 8897.744327838274 * pbar2 + -333447.9532187822 * pbar4 + 4543176.674460011 * pbar6)  + Cm3 * (5264.381555481536 * pbar0 + -158683.96989838342 * pbar2 + 3967573.184636821 * pbar4 + -32945776.33726536 * pbar6)  + Cm4 * (-11344.862415692254 * pbar0 + -442802.89444108633 * pbar2 + 25911831.53766136 * pbar4 + -384475889.703549 * pbar6)  + Cm5 * (-208361.5559122354 * pbar0 + 10189909.415514922 * pbar2 + -230861692.01471433 * pbar4 + 1326183541.253531 * pbar6) )  + CL1 * (Cm0 * (-16.862890646667285 * pbar0 + 16.691023133015445 * pbar2 + 552.1222258546513 * pbar4 + 19952.316872079315 * pbar6)  + Cm1 * (159.11017499467 * pbar0 + 963.5937389928912 * pbar2 + -10298.481899657809 * pbar4 + 4300.964202460459 * pbar6)  + Cm2 * (1171.321750639268 * pbar0 + -57319.75991997871 * pbar2 + 2888842.3159483336 * pbar4 + -46170851.027685955 * pbar6)  + Cm3 * (-39914.18975130367 * pbar0 + 1369060.4072674918 * pbar2 + -37701239.39168474 * pbar4 + 319555638.6518483 * pbar6)  + Cm4 * (20528.257783740948 * pbar0 + 5786274.004331231 * pbar2 + -325259544.58144003 * pbar4 + 4360140363.331053 * pbar6)  + Cm5 * (1927674.3535650226 * pbar0 + -86852343.40381642 * pbar2 + 1785037754.260697 * pbar4 + -11333608494.99297 * pbar6) )  + CL2 * (Cm0 * (12.642296980258832 * pbar0 + 381.78647248548134 * pbar2 + -8073.439651285395 * pbar4 + -3919.4086074849565 * pbar6)  + Cm1 * (-158.44682284172836 * pbar0 + 2080.0534010498295 * pbar2 + -172341.0226648455 * pbar4 + 1993198.4152621645 * pbar6)  + Cm2 * (-2855.5262331464623 * pbar0 + 117545.19826674717 * pbar2 + -7333208.610078409 * pbar4 + 113823117.27764262 * pbar6)  + Cm3 * (80115.9980933689 * pbar0 + -3438547.3145430237 * pbar2 + 110259324.88649 * pbar4 + -1216460911.414372 * pbar6)  + Cm4 * (23619.30069114353 * pbar0 + -16297422.976653978 * pbar2 + 844574658.858567 * pbar4 + -10366360451.167824 * pbar6)  + Cm5 * (-4286318.703715176 * pbar0 + 206298668.84514797 * pbar2 + -5666151712.549294 * pbar4 + 68072491517.90248 * pbar6) )  + CL3 * (Cm0 * (-4.484429764604437 * pbar0 + -298.8599290210548 * pbar2 + 5317.787342696812 * pbar4 + 1653.8687047892856 * pbar6)  + Cm1 * (72.30972470590153 * pbar0 + -4661.722740124539 * pbar2 + 222472.1207251823 * pbar4 + -2542462.8556045797 * pbar6)  + Cm2 * (1754.545925810411 * pbar0 + -87837.7263251788 * pbar2 + 5378302.6201214455 * pbar4 + -77858278.01263912 * pbar6)  + Cm3 * (-47698.54906935265 * pbar0 + 2416152.3687686175 * pbar2 + -88860913.27203426 * pbar4 + 1160677507.8556125 * pbar6)  + Cm4 * (-37884.21841792403 * pbar0 + 11904232.931940025 * pbar2 + -578140833.1207916 * pbar4 + 6712457655.532777 * pbar6)  + Cm5 * (2693342.2813032027 * pbar0 + -143255870.98400217 * pbar2 + 5097882976.428307 * pbar4 + -76752184639.2084 * pbar6) ) 
    
    sym[1] = CL0 * (Cm0 * (-2.2295033562764535 * pbar0 + -74.06787015304576 * pbar2 + 2743.1590269624908 * pbar4 + -34714.60253029948 * pbar6)  + Cm1 * (-65.12019545391139 * pbar0 + 5195.924363318838 * pbar2 + -208056.74015047515 * pbar4 + 2867949.463972608 * pbar6)  + Cm2 * (-155.17003904245985 * pbar0 + 29229.685286544907 * pbar2 + -1333437.1933692426 * pbar4 + 17276017.860765114 * pbar6)  + Cm3 * (7040.34257706067 * pbar0 + -1175574.053632094 * pbar2 + 60678712.04800408 * pbar4 + -918819587.3971224 * pbar6)  + Cm4 * (4024.925664603419 * pbar0 + -1483734.6815027 * pbar2 + 71586154.49192251 * pbar4 + -949346687.3875941 * pbar6)  + Cm5 * (-348086.97104815854 * pbar0 + 70970659.32881841 * pbar2 + -3961892211.6898603 * pbar4 + 62330626438.91596 * pbar6) )  + CL1 * (Cm0 * (-7.809787528227156 * pbar0 + 1366.9523786942289 * pbar2 + -49064.71981507403 * pbar4 + 576223.7410943059 * pbar6)  + Cm1 * (146.17554993304861 * pbar0 + -23780.708967770162 * pbar2 + 1431035.898785872 * pbar4 + -23753069.781376526 * pbar6)  + Cm2 * (3398.886590634059 * pbar0 + -504352.3615124907 * pbar2 + 20608820.426515214 * pbar4 + -254931174.75455746 * pbar6)  + Cm3 * (-52347.32224550817 * pbar0 + 10193339.223122882 * pbar2 + -589277242.7676378 * pbar4 + 9520366941.294786 * pbar6)  + Cm4 * (-145617.1444315727 * pbar0 + 25976281.214089222 * pbar2 + -1118087077.327236 * pbar4 + 14144787617.136139 * pbar6)  + Cm5 * (3188442.37887667 * pbar0 + -720059928.3036283 * pbar2 + 42986611866.94436 * pbar4 + -703166007902.1171 * pbar6) )  + CL2 * (Cm0 * (18.7551548826787 * pbar0 + -2325.369326168607 * pbar2 + 103411.24163998842 * pbar4 + -1317146.9399178869 * pbar6)  + Cm1 * (-82.63743208109965 * pbar0 + 24212.857007508486 * pbar2 + -2368760.4561385983 * pbar4 + 46439894.52171445 * pbar6)  + Cm2 * (-8001.028258822571 * pbar0 + 1193427.4174999702 * pbar2 + -50164703.11615148 * pbar4 + 633685674.8231791 * pbar6)  + Cm3 * (97878.48308563221 * pbar0 + -21443827.38905333 * pbar2 + 1336911924.9046378 * pbar4 + -22454810884.95594 * pbar6)  + Cm4 * (411666.0571915268 * pbar0 + -66972088.385635376 * pbar2 + 2860152865.924816 * pbar4 + -36396385195.95659 * pbar6)  + Cm5 * (-6950459.237998394 * pbar0 + 1685762717.5134587 * pbar2 + -104500532000.20956 * pbar4 + 1742616152657.2407 * pbar6) )  + CL3 * (Cm0 * (-10.43636129198971 * pbar0 + 1037.924567964284 * pbar2 + -55276.81480333282 * pbar4 + 761065.8935153007 * pbar6)  + Cm1 * (-9.565740218111989 * pbar0 + -6048.914243397562 * pbar2 + 1168050.8170788018 * pbar4 + -26234504.851764325 * pbar6)  + Cm2 * (4870.6045958404875 * pbar0 + -724510.7146796053 * pbar2 + 31389826.188067015 * pbar4 + -406490580.78407055 * pbar6)  + Cm3 * (-53777.93082565168 * pbar0 + 12917695.976419926 * pbar2 + -846545796.1552124 * pbar4 + 14565403538.05362 * pbar6)  + Cm4 * (-281919.73371705104 * pbar0 + 43934257.99903655 * pbar2 + -1884321400.0994751 * pbar4 + 24280566917.268215 * pbar6)  + Cm5 * (4270690.243169417 * pbar0 + -1090815329.1665342 * pbar2 + 69210396455.93594 * pbar4 + -1166912801133.8142 * pbar6) ) 
    
    sym[2] = CL0 * (Cm0 * (-2.5368588451234917 * pbar0 + -50.69376520020511 * pbar2 + 1681.876557523829 * pbar4 + -22799.40837905926 * pbar6)  + Cm1 * (-55.55232784404335 * pbar0 + 728.4942307291598 * pbar2 + 16680.962419172418 * pbar4 + -454372.95554310683 * pbar6)  + Cm2 * (-91.24255958914648 * pbar0 + 17178.607899449362 * pbar2 + -830248.2331714694 * pbar4 + 12309411.334685607 * pbar6)  + Cm3 * (2387.042173547277 * pbar0 + -70149.91869062658 * pbar2 + -3878548.4370314595 * pbar4 + 119845824.72719422 * pbar6)  + Cm4 * (1780.9230314857741 * pbar0 + -867974.761835413 * pbar2 + 51647802.6788524 * pbar4 + -810635705.4495621 * pbar6)  + Cm5 * (-103115.41000968656 * pbar0 + 1849606.7088422505 * pbar2 + 384632226.3823886 * pbar4 + -10112149502.058174 * pbar6) )  + CL1 * (Cm0 * (-5.619822902496444 * pbar0 + 423.2468983573377 * pbar2 + -15992.662406336198 * pbar4 + 236724.17805206164 * pbar6)  + Cm1 * (-11.352470061849395 * pbar0 + 11076.761102179957 * pbar2 + -575424.9677621159 * pbar4 + 8580030.463885976 * pbar6)  + Cm2 * (1360.0230519031034 * pbar0 + -171686.4698635674 * pbar2 + 8730526.435097214 * pbar4 + -131277845.80135365 * pbar6)  + Cm3 * (-11896.63981260938 * pbar0 + -448574.9847479719 * pbar2 + 80563303.15673801 * pbar4 + -1690286399.3914087 * pbar6)  + Cm4 * (-48949.633425569686 * pbar0 + 10859566.629770182 * pbar2 + -607615805.1317221 * pbar4 + 8990858938.04455 * pbar6)  + Cm5 * (804527.3043499555 * pbar0 + 14554984.451975837 * pbar2 + -5246211839.903592 * pbar4 + 115385780103.2795 * pbar6) )  + CL2 * (Cm0 * (1.6705941707099778 * pbar0 + 89.87060264227638 * pbar2 + 15471.038750017238 * pbar4 + -375043.92760328576 * pbar6)  + Cm1 * (137.22548796604272 * pbar0 + -34621.19386417567 * pbar2 + 1713916.964488748 * pbar4 + -25025920.325223245 * pbar6)  + Cm2 * (-2240.3847756197647 * pbar0 + 299373.7813751692 * pbar2 + -17899673.532257956 * pbar4 + 283202098.9196727 * pbar6)  + Cm3 * (17642.37630228904 * pbar0 + 2081551.1099600054 * pbar2 + -232657572.20970818 * pbar4 + 4459969496.51816 * pbar6)  + Cm4 * (122882.43000594882 * pbar0 + -25692671.396869153 * pbar2 + 1417980126.6806028 * pbar4 + -20641802763.586727 * pbar6)  + Cm5 * (-1633348.3497929808 * pbar0 + -66489018.72281311 * pbar2 + 13216149486.806889 * pbar4 + -271705039546.30774 * pbar6) )  + CL3 * (Cm0 * (1.140560174041999 * pbar0 + -418.4917159651862 * pbar2 + 685.426441915029 * pbar4 + 133373.87395655087 * pbar6)  + Cm1 * (-82.94689671238417 * pbar0 + 22105.149278519315 * pbar2 + -1184667.601306534 * pbar4 + 17688394.541102238 * pbar6)  + Cm2 * (1031.623178200886 * pbar0 + -144736.23843745745 * pbar2 + 9887698.611640822 * pbar4 + -163797653.63569176 * pbar6)  + Cm3 * (-8836.006313466327 * pbar0 + -1601049.9628873086 * pbar2 + 161990041.3600831 * pbar4 + -3000219002.5559483 * pbar6)  + Cm4 * (-78793.27139617878 * pbar0 + 16268287.523857692 * pbar2 + -886036400.9571072 * pbar4 + 12833482384.332502 * pbar6)  + Cm5 * (977195.3496735883 * pbar0 + 50320966.61653907 * pbar2 + -8483229831.372911 * pbar4 + 168896174152.96808 * pbar6) ) 
    
    sym[3] = CL0 * (Cm0 * (-2.9001273576720994 * pbar0 + -56.90310573275944 * pbar2 + 2054.368044623417 * pbar4 + -29714.15017915107 * pbar6)  + Cm1 * (-56.76794302193099 * pbar0 + -459.8319546127149 * pbar2 + 49986.867180272326 * pbar4 + -837382.8833329666 * pbar6)  + Cm2 * (-81.95831813124393 * pbar0 + 16270.348180417677 * pbar2 + -961744.788795834 * pbar4 + 15981193.935371008 * pbar6)  + Cm3 * (-380.1443278776027 * pbar0 + 193538.29595029654 * pbar2 + -14505984.449821835 * pbar4 + 274389274.4969356 * pbar6)  + Cm4 * (2491.032242149016 * pbar0 + -855609.7380000304 * pbar2 + 68279412.97223528 * pbar4 + -1183591862.9747164 * pbar6)  + Cm5 * (26610.362164833583 * pbar0 + -15114119.431448668 * pbar2 + 1232696921.9165914 * pbar4 + -23373984914.593483 * pbar6) )  + CL1 * (Cm0 * (-7.896184189231455 * pbar0 + 221.23593564402177 * pbar2 + -13435.909055809398 * pbar4 + 252813.94123757063 * pbar6)  + Cm1 * (-109.51196799336194 * pbar0 + 15986.707855774655 * pbar2 + -680335.2240960692 * pbar4 + 10216627.440182572 * pbar6)  + Cm2 * (338.7061054859648 * pbar0 + -101683.70223274411 * pbar2 + 9601345.440519115 * pbar4 + -170263902.97535762 * pbar6)  + Cm3 * (10323.404743128109 * pbar0 + -2403933.6956556337 * pbar2 + 173685557.22046053 * pbar4 + -3208144341.0695233 * pbar6)  + Cm4 * (-6175.782087808313 * pbar0 + 9302414.499941405 * pbar2 + -803440787.8043464 * pbar4 + 13064124687.625574 * pbar6)  + Cm5 * (-362512.95427824394 * pbar0 + 177350156.72576505 * pbar2 + -14386984707.855272 * pbar4 + 261637557809.9441 * pbar6) )  + CL2 * (Cm0 * (-5.5897838997487606 * pbar0 + 412.627751851462 * pbar2 + 19791.30222582614 * pbar4 + -549629.2168281592 * pbar6)  + Cm1 * (284.1309618567934 * pbar0 + -35360.75799997247 * pbar2 + 1647106.23411247 * pbar4 + -25788746.592548184 * pbar6)  + Cm2 * (736.6704835759028 * pbar0 + 174616.84290800357 * pbar2 + -23393413.653277546 * pbar4 + 416219776.8386022 * pbar6)  + Cm3 * (-23919.32219353151 * pbar0 + 5609779.440332826 * pbar2 + -426450051.9709869 * pbar4 + 7722828081.268656 * pbar6)  + Cm4 * (-15020.629320212904 * pbar0 + -24750583.10147274 * pbar2 + 2038993449.2574768 * pbar4 + -31570868975.140873 * pbar6)  + Cm5 * (829541.5207885458 * pbar0 + -430657375.1718625 * pbar2 + 34270404044.768402 * pbar4 + -601594073326.0936 * pbar6) )  + CL3 * (Cm0 * (6.744682027857709 * pbar0 + -494.6191628450697 * pbar2 + -9465.924619458769 * pbar4 + 337909.5200083927 * pbar6)  + Cm1 * (-139.02891601690413 * pbar0 + 18907.664377260116 * pbar2 + -1055923.3272002544 * pbar4 + 17256399.232295424 * pbar6)  + Cm2 * (-950.2327682656975 * pbar0 + -110338.82402677562 * pbar2 + 15800926.982400639 * pbar4 + -276817939.0491136 * pbar6)  + Cm3 * (13304.74066873803 * pbar0 + -3488294.184701939 * pbar2 + 274996626.2224565 * pbar4 + -4876032533.695871 * pbar6)  + Cm4 * (19038.4319419409 * pbar0 + 17784605.61906063 * pbar2 + -1383018016.9339056 * pbar4 + 20834667918.428047 * pbar6)  + Cm5 * (-484565.50893541006 * pbar0 + 274820202.32876676 * pbar2 + -21361038326.6559 * pbar4 + 363842402968.69086 * pbar6) ) 
    
    sym[4] = CL0 * (Cm0 * (-3.05757869956143 * pbar0 + -49.639159117545816 * pbar2 + 847.1565486483444 * pbar4 + -7331.481937321558 * pbar6)  + Cm1 * (-64.88452389190248 * pbar0 + 1920.0862086029204 * pbar2 + -115247.29935032292 * pbar4 + 1827670.751578467 * pbar6)  + Cm2 * (-69.99222601234507 * pbar0 + -857.9858957702373 * pbar2 + 65604.44610950565 * pbar4 + 873333.8067448139 * pbar6)  + Cm3 * (-382.90919351341074 * pbar0 + -470436.75764121575 * pbar2 + 34690550.303666115 * pbar4 + -558203302.9012895 * pbar6)  + Cm4 * (1647.4789531269687 * pbar0 + 659795.3481528505 * pbar2 + -16305465.230338972 * pbar4 + -77242942.2669146 * pbar6)  + Cm5 * (-2738.3930443009663 * pbar0 + 31118250.773247316 * pbar2 + -2209546427.157859 * pbar4 + 34812317957.90519 * pbar6) )  + CL1 * (Cm0 * (-11.741920141939733 * pbar0 + 195.58335755902021 * pbar2 + -3004.3949868932045 * pbar4 + 33013.204245211244 * pbar6)  + Cm1 * (-75.55514691595775 * pbar0 + -13859.098592462895 * pbar2 + 1216996.2791880302 * pbar4 + -19904550.976047967 * pbar6)  + Cm2 * (-168.85505516167768 * pbar0 + 76995.61159088068 * pbar2 + -1657652.6912967826 * pbar4 + -10422770.821276776 * pbar6)  + Cm3 * (4257.071387732378 * pbar0 + 5506944.014653135 * pbar2 + -393801989.4701221 * pbar4 + 6171531778.601536 * pbar6)  + Cm4 * (25092.10274683965 * pbar0 + -8444585.646012323 * pbar2 + 108715091.90659012 * pbar4 + 2922422760.8694425 * pbar6)  + Cm5 * (129387.81236419089 * pbar0 + -366549636.60371935 * pbar2 + 24539170075.417763 * pbar4 + -371115393059.4506 * pbar6) )  + CL2 * (Cm0 * (-4.309499701385465 * pbar0 + 62.82609535176756 * pbar2 + 4741.381044355423 * pbar4 + -118452.79555178845 * pbar6)  + Cm1 * (156.31628389634542 * pbar0 + 40654.104036712386 * pbar2 + -3309325.6510033803 * pbar4 + 52963499.32823226 * pbar6)  + Cm2 * (1696.1975363309184 * pbar0 + -216317.87378488298 * pbar2 + 941751.1031628866 * pbar4 + 98594513.58498278 * pbar6)  + Cm3 * (-1987.1765988273457 * pbar0 + -15522973.262827728 * pbar2 + 1049080958.3398789 * pbar4 + -15968501710.70516 * pbar6)  + Cm4 * (-104574.96512702739 * pbar0 + 16368214.03467465 * pbar2 + 260350047.83609727 * pbar4 + -16822683811.097889 * pbar6)  + Cm5 * (-703120.6092878161 * pbar0 + 990640704.0556301 * pbar2 + -62855522584.97492 * pbar4 + 919603573277.6741 * pbar6) )  + CL3 * (Cm0 * (5.407272711473305 * pbar0 + -134.8911330766212 * pbar2 + -4808.597858052572 * pbar4 + 133184.11566243772 * pbar6)  + Cm1 * (-53.68026592183247 * pbar0 + -33001.389545137936 * pbar2 + 2447856.770833651 * pbar4 + -38237774.214278504 * pbar6)  + Cm2 * (-1462.3806520503392 * pbar0 + 114805.85964237833 * pbar2 + 3210660.4584812415 * pbar4 + -134848089.4871759 * pbar6)  + Cm3 * (-4477.798765234236 * pbar0 + 11614773.00254374 * pbar2 + -748290422.4243308 * pbar4 + 11121893667.469328 * pbar6)  + Cm4 * (79710.26049511954 * pbar0 + -5410047.632169888 * pbar2 + -622488254.3746046 * pbar4 + 18685781679.237736 * pbar6)  + Cm5 * (705953.4599722155 * pbar0 + -710561837.3856395 * pbar2 + 43236181983.57092 * pbar4 + -616206027392.0244 * pbar6) ) 
    
    sym[5] = CL0 * (Cm0 * (-3.912622898390996 * pbar0 + 49.33207798186015 * pbar2 + -4921.092699649912 * pbar4 + 86677.46742731953 * pbar6)  + Cm1 * (-93.44065837115114 * pbar0 + 7218.117869480467 * pbar2 + -280061.45291833795 * pbar4 + 3050778.231552465 * pbar6)  + Cm2 * (-8.275520763092148 * pbar0 + -30917.987736336472 * pbar2 + 2243230.99725415 * pbar4 + -41183501.48406109 * pbar6)  + Cm3 * (2432.000461557786 * pbar0 + -728444.0104560126 * pbar2 + 22893969.895566802 * pbar4 + -152615438.44966948 * pbar6)  + Cm4 * (3980.1905392538256 * pbar0 + 1282638.4909151471 * pbar2 + -111530318.44491209 * pbar4 + 2414467502.6005516 * pbar6)  + Cm5 * (-81161.95903868933 * pbar0 + 9894662.196975833 * pbar2 + 362525918.84872407 * pbar4 + -12930289527.221928 * pbar6) )  + CL1 * (Cm0 * (-15.514568535956936 * pbar0 + 578.4895206536447 * pbar2 + -730.8338028911102 * pbar4 + -268336.1607153741 * pbar6)  + Cm1 * (189.4344050622744 * pbar0 + -69631.93467590187 * pbar2 + 2906511.999794063 * pbar4 + -32398570.272145174 * pbar6)  + Cm2 * (985.8910252532944 * pbar0 + -200689.13430328885 * pbar2 + 1416163.9945777052 * pbar4 + 124300317.43510625 * pbar6)  + Cm3 * (-27529.262044041556 * pbar0 + 7258045.629180533 * pbar2 + -218645802.7511858 * pbar4 + 1628804009.77525 * pbar6)  + Cm4 * (-99223.90405469749 * pbar0 + 15095538.294594016 * pbar2 + -84633072.25290354 * pbar4 + -10375697344.347551 * pbar6)  + Cm5 * (891065.3674109961 * pbar0 + -94675927.53095853 * pbar2 + -2640348952.3683076 * pbar4 + 84760923106.17082 * pbar6) )  + CL2 * (Cm0 * (7.016439012907092 * pbar0 + -2844.2647499018894 * pbar2 + 74842.13012175163 * pbar4 + -174617.96422376664 * pbar6)  + Cm1 * (-438.20922183350353 * pbar0 + 144906.0366842942 * pbar2 + -5915661.739116525 * pbar4 + 66002814.04625338 * pbar6)  + Cm2 * (-4084.122276836624 * pbar0 + 1156387.4838664592 * pbar2 + -29914478.492668908 * pbar4 + -16508553.074209683 * pbar6)  + Cm3 * (60414.508556888744 * pbar0 + -12938927.207969302 * pbar2 + 320331989.1485334 * pbar4 + -1778869774.2855506 * pbar6)  + Cm4 * (335088.59960324236 * pbar0 + -63427136.51324534 * pbar2 + 1032613049.5604719 * pbar4 + 18650983859.94038 * pbar6)  + Cm5 * (-1805810.695222171 * pbar0 + 81053494.33399361 * pbar2 + 10825152828.829681 * pbar4 + -214513705637.5281 * pbar6) )  + CL3 * (Cm0 * (-5.6621069373753246 * pbar0 + 2619.1042480242945 * pbar2 + -80091.53605159458 * pbar4 + 453481.51326700434 * pbar6)  + Cm1 * (275.3554513634697 * pbar0 + -76317.2251428587 * pbar2 + 3045177.000716673 * pbar4 + -33923176.34728389 * pbar6)  + Cm2 * (3633.020525336429 * pbar0 + -987787.6522911284 * pbar2 + 26895181.32029161 * pbar4 + -47634589.561838016 * pbar6)  + Cm3 * (-32907.33610353668 * pbar0 + 5362494.772064799 * pbar2 + -71915645.13282435 * pbar4 + -373838440.8706798 * pbar6)  + Cm4 * (-257315.3153474657 * pbar0 + 47388609.55603383 * pbar2 + -664737604.4089582 * pbar4 + -15197017528.164383 * pbar6)  + Cm5 * (919635.7861907539 * pbar0 + 44991027.55805685 * pbar2 + -10633535845.48275 * pbar4 + 167483678506.46313 * pbar6) ) 
    
    asym[0] = CL0 * (Cm0 * (-0.0007358457722695937 * pbar0 + -20.196061628947465 * pbar1 + 0.03007134579233035 * pbar2 + -95.24430070404799 * pbar3)  + Cm1 * (-0.022857054494426064 * pbar0 + 5.16875746201123 * pbar1 + 0.8922982096478065 * pbar2 + 381.35877138115023 * pbar3)  + Cm2 * (0.18763747709566686 * pbar0 + -327.1816757931467 * pbar1 + -7.811884503668922 * pbar2 + 14984.687983811687 * pbar3) )  + CL1 * (Cm0 * (0.009318402310135173 * pbar0 + -7.8843805367130075 * pbar1 + -0.40319127071948757 * pbar2 + 549.303089272052 * pbar3)  + Cm1 * (0.2935085518821942 * pbar0 + -327.71069266932005 * pbar1 + -11.50449288541838 * pbar2 + 10215.156889665182 * pbar3)  + Cm2 * (-2.300007615591028 * pbar0 + 3387.9277758291564 * pbar1 + 96.82504420893918 * pbar2 + -105093.88884975197 * pbar3) )  + CL2 * (Cm0 * (-0.024951726343310922 * pbar0 + -29.833710619570756 * pbar1 + 1.1010348052325798 * pbar2 + 31.415374606549737 * pbar3)  + Cm1 * (-0.7661909945706882 * pbar0 + 1025.3525787622698 * pbar1 + 30.09307303625841 * pbar2 + -32636.138243829086 * pbar3)  + Cm2 * (7.004593020646519 * pbar0 + -1106.1017388074151 * pbar1 + -293.64093671914173 * pbar2 + 25305.51884088407 * pbar3) )  + CL3 * (Cm0 * (0.01718353287353797 * pbar0 + 41.384044406353546 * pbar1 + -0.7652351627926158 * pbar2 + -628.0385950464006 * pbar3)  + Cm1 * (0.5195929026624577 * pbar0 + -606.6339669084198 * pbar1 + -20.446341341266617 * pbar2 + 20905.599419376933 * pbar3)  + Cm2 * (-5.178776713246014 * pbar0 + -2809.8672860544248 * pbar1 + 216.4886788576724 * pbar2 + 92202.9653372512 * pbar3) ) 
    
    asym[1] = CL0 * (Cm0 * (-0.0005343899989459703 * pbar0 + -35.80504744591387 * pbar1 + 0.022221415855260163 * pbar2 + -62.56374620698434 * pbar3)  + Cm1 * (-0.023073147448584234 * pbar0 + -13.76036683349285 * pbar1 + 0.9605708814313393 * pbar2 + 694.220100750486 * pbar3)  + Cm2 * (0.09858734888352504 * pbar0 + -305.9916894677363 * pbar1 + -4.0457064652578 * pbar2 + 20811.118378565992 * pbar3) )  + CL1 * (Cm0 * (0.006826382262578586 * pbar0 + -27.48796180825012 * pbar1 + -0.27476386058108826 * pbar2 + 1019.3322513562939 * pbar3)  + Cm1 * (0.30250522195643487 * pbar0 + -289.3294243712582 * pbar1 + -12.555042373434139 * pbar2 + 12036.427525853425 * pbar3)  + Cm2 * (-1.1395915364409257 * pbar0 + 5882.312920050401 * pbar1 + 45.363799061738874 * pbar2 + -187568.5940463473 * pbar3) )  + CL2 * (Cm0 * (-0.018527147681767223 * pbar0 + 15.956780717750457 * pbar1 + 0.7451066307818561 * pbar2 + -798.9983730678798 * pbar3)  + Cm1 * (-0.7757454779250323 * pbar0 + 1398.267042351697 * pbar1 + 32.268566905446384 * pbar2 + -45399.33023997115 * pbar3)  + Cm2 * (3.950652336201692 * pbar0 + -6004.69057457365 * pbar1 + -157.6285957403321 * pbar2 + 152580.52070301736 * pbar3) )  + CL3 * (Cm0 * (0.012801971398744007 * pbar0 + 23.422974681272848 * pbar1 + -0.5110961256653117 * pbar2 + -413.205841618814 * pbar3)  + Cm1 * (0.5199152141080706 * pbar0 + -950.3792191063785 * pbar1 + -21.69157536900651 * pbar2 + 30887.77080186874 * pbar3)  + Cm2 * (-3.076367654356798 * pbar0 + -833.0409632193189 * pbar1 + 122.7189612638105 * pbar2 + 54361.88278373735 * pbar3) ) 
    
    asym[2] = CL0 * (Cm0 * (3.0379117402658574e-05 * pbar0 + -48.30551316231284 * pbar1 + -0.000481749285565671 * pbar2 + 61.7428135184152 * pbar3)  + Cm1 * (-0.003185154790565774 * pbar0 + -55.65598576806201 * pbar1 + 0.11498679830694855 * pbar2 + 1113.701722422874 * pbar3)  + Cm2 * (-0.06260351017593548 * pbar0 + 212.64043804466635 * pbar1 + 2.567384517792957 * pbar2 + 6438.071848232091 * pbar3) )  + CL1 * (Cm0 * (-0.0004043649697535394 * pbar0 + -33.61407307610878 * pbar1 + 0.006981404278218288 * pbar2 + 923.5826972944055 * pbar3)  + Cm1 * (0.03788709254328494 * pbar0 + 197.823584513754 * pbar1 + -1.4178728775932277 * pbar2 + -110.71383691535434 * pbar3)  + Cm2 * (0.8298473336977951 * pbar0 + 4639.031267099806 * pbar1 + -33.952547806005065 * pbar2 + -150330.05615798876 * pbar3) )  + CL2 * (Cm0 * (0.0014354277568028367 * pbar0 + 76.84860091189793 * pbar1 + -0.04429158284838754 * pbar2 + -1757.6256215174478 * pbar3)  + Cm1 * (-0.07652915953994398 * pbar0 + 559.6742438364596 * pbar1 + 2.8862093353985157 * pbar2 + -21029.92111383806 * pbar3)  + Cm2 * (-2.143919535764161 * pbar0 + -8528.019034522067 * pbar1 + 90.05560888578185 * pbar2 + 216011.15156722328 * pbar3) )  + CL3 * (Cm0 * (-0.0011428710907679 * pbar0 + -24.883461907432572 * pbar1 + 0.03988034644516888 * pbar2 + 501.8250760142316 * pbar3)  + Cm1 * (0.04277225606540764 * pbar0 + -604.1332648097995 * pbar1 + -1.6296193910504821 * pbar2 + 19187.116265445584 * pbar3)  + Cm2 * (1.4417289674576457 * pbar0 + 2343.6634889117745 * pbar1 + -61.68774194244056 * pbar2 + -38727.70254912978 * pbar3) ) 
    
    asym[3] = CL0 * (Cm0 * (0.0010099595144260082 * pbar0 + -56.85673888710481 * pbar1 + -0.04133140225346068 * pbar2 + 275.2234277294981 * pbar3)  + Cm1 * (0.03734965098161625 * pbar0 + -109.80999432318454 * pbar1 + -1.5153280408934449 * pbar2 + 1484.8211063906765 * pbar3)  + Cm2 * (-0.2213483347200175 * pbar0 + 1164.055102629567 * pbar1 + 8.962155702208584 * pbar2 + -31881.11393986062 * pbar3) )  + CL1 * (Cm0 * (-0.012626912814483518 * pbar0 + -6.728893707647991 * pbar1 + 0.5285026803353366 * pbar2 + -357.2123863921613 * pbar3)  + Cm1 * (-0.47593277901174424 * pbar0 + 845.2578837370239 * pbar1 + 19.309962706808626 * pbar2 + -21880.889914173218 * pbar3)  + Cm2 * (2.6174263449696724 * pbar0 + -4028.6857484245347 * pbar1 + -105.95179158536199 * pbar2 + 127615.67435288016 * pbar3) )  + CL2 * (Cm0 * (0.033665054166198284 * pbar0 + 70.74689628257623 * pbar1 + -1.4188609904061904 * pbar2 + -1147.9511337623753 * pbar3)  + Cm1 * (1.2242660558837226 * pbar0 + -1461.5721026924375 * pbar1 + -49.765497147085085 * pbar2 + 51727.560144100724 * pbar3)  + Cm2 * (-8.286070684104091 * pbar0 + -436.41613731227477 * pbar1 + 333.70316029100474 * pbar2 + 77858.06936450145 * pbar3) )  + CL3 * (Cm0 * (-0.023075849043163475 * pbar0 + -56.1853938792524 * pbar1 + 0.9719199636122348 * pbar2 + 1432.033226654999 * pbar3)  + Cm1 * (-0.8231446854689498 * pbar0 + 780.775668410447 * pbar1 + 33.538729853116934 * pbar2 + -29181.962130653817 * pbar3)  + Cm2 * (6.222583409400713 * pbar0 + 4356.754174130889 * pbar1 + -249.30607052045679 * pbar2 + -192707.2626269142 * pbar3) ) 
    
    asym[4] = CL0 * (Cm0 * (0.0019438070360623901 * pbar0 + -68.986418534107 * pbar1 + -0.07753867814140683 * pbar2 + 669.3449267754725 * pbar3)  + Cm1 * (0.08316619127504558 * pbar0 + -109.84381987320306 * pbar1 + -3.2850371233782134 * pbar2 + 817.303175707521 * pbar3)  + Cm2 * (-0.3724473547769404 * pbar0 + 3169.593444852714 * pbar1 + 14.736493877843506 * pbar2 + -106529.61267806444 * pbar3) )  + CL1 * (Cm0 * (-0.02505542387489638 * pbar0 + 77.6466789021366 * pbar1 + 0.9978638060623009 * pbar2 + -3795.067216365176 * pbar3)  + Cm1 * (-1.0897257016279573 * pbar0 + 1344.7788327758885 * pbar1 + 43.10791033875242 * pbar2 + -36060.10723594586 * pbar3)  + Cm2 * (4.367588606414232 * pbar0 + -24915.619531318134 * pbar1 + -172.5674259202431 * pbar2 + 966787.7639741619 * pbar3) )  + CL2 * (Cm0 * (0.06832087922845755 * pbar0 + -90.16561047891126 * pbar1 + -2.7078888354892983 * pbar2 + 5590.631520228269 * pbar3)  + Cm1 * (2.789630560433429 * pbar0 + -3134.7743089349965 * pbar1 + -110.36716547183521 * pbar2 + 113653.9339117832 * pbar3)  + Cm2 * (-14.909930482406661 * pbar0 + 44876.53764961867 * pbar1 + 586.5125408573516 * pbar2 + -1671161.9477394137 * pbar3) )  + CL3 * (Cm0 * (-0.04752647134182442 * pbar0 + 25.644994647360242 * pbar1 + 1.880474877034055 * pbar2 + -1857.0989833099968 * pbar3)  + Cm1 * (-1.8682752964235378 * pbar0 + 2197.782653697633 * pbar1 + 73.92935633242978 * pbar2 + -82371.85475817164 * pbar3)  + Cm2 * (11.590948821062662 * pbar0 + -19429.378881605186 * pbar1 + -455.51926202219823 * pbar2 + 691618.8303751013 * pbar3) )
    
    return boundActuators(sym, asym)

############################################################################
############################################################################
############################################################################

def mode1Trim(control=True):
    '''
    if control is true
        returns throttle, roll, pitch commands to trim Horizon in mode 1
    otherwise
        returns aoa (deg) for mode 1 trim condition
    '''
    if control: return 0.099875095666051114, 0.0, -0.431837400614613287
    return 11.266741443177069826

def mode2Trim(control=True):
    
    if control: return 0.0865005550194539, 0.4840267710585861, 0.0, 0.0283044393279927
    return 8.5725791295286413

