package frc.lib;

import java.util.ArrayList;
import frc.lib.*;
// import com.ctre.phoenix.motion.TrajectoryPoint;
public class CreateNewPath {
    public static boolean restart;
    public static double[][] pathLeft = new double[][] {
        {0,0,0,0.0001},
        {0.0004,0.0407,3.7166,0.0003},
        {0.0012,0.0778,3.8858,0.0007},
        {0.0024,0.1249,4.9531,0.0015},
        {0.0042,0.1737,4.7619,0.0026},
        {0.0064,0.2235,4.999,0.0042},
        {0.009,0.2722,4.9724,0.006},
        {0.0123,0.3215,4.8601,0.0083},
        {0.016,0.3712,4.9657,0.011},
        {0.0202,0.4202,4.9632,0.0139},
        {0.0249,0.4697,4.891,0.0173},
        {0.0301,0.5193,4.9556,0.021},
        {0.0358,0.5686,4.9598,0.0251},
        {0.042,0.6182,4.9081,0.0295},
        {0.0486,0.6673,4.998,0.0342},
        {0.0557,0.7164,4.9215,0.0391},
        {0.0635,0.7661,4.9116,0.0445},
        {0.0715,0.8154,4.9896,0.0501},
        {0.0802,0.8648,4.934,0.056},
        {0.0895,0.9147,4.9225,0.0623},
        {0.099,0.9642,4.9863,0.0688},
        {0.1092,1.0139,4.9446,0.0755},
        {0.1197,1.0631,4.9841,0.0825},
        {0.131,1.1129,4.9251,0.0897},
        {0.1426,1.1625,4.9778,0.0971},
        {0.1546,1.2119,4.9791,0.1047},
        {0.1673,1.2616,4.9509,0.1125},
        {0.1804,1.3114,4.9676,0.1204},
        {0.194,1.3611,4.9777,0.1286},
        {0.2081,1.4108,4.9797,0.1368},
        {0.2228,1.4607,4.9639,0.1452},
        {0.2378,1.5104,4.9953,0.1537},
        {0.2535,1.5604,4.9774,0.1623},
        {0.2694,1.6101,4.9991,0.171},
        {0.2861,1.66,4.9832,0.1797},
        {0.3032,1.7099,4.9883,0.1884},
        {0.3208,1.7599,4.9981,0.1972},
        {0.3389,1.8099,5.0009,0.206},
        {0.3575,1.8599,5.0037,0.2148},
        {0.3766,1.9099,5.0067,0.2235},
        {0.3961,1.96,5.0096,0.2322},
        {0.4162,2.0101,5.0126,0.2408},
        {0.4368,2.0602,5.0157,0.2493},
        {0.4579,2.1104,5.0188,0.2577},
        {0.4795,2.1606,5.022,0.2659},
        {0.5016,2.2109,5.0252,0.2741},
        {0.5243,2.2612,5.0284,0.282},
        {0.5474,2.3116,5.0316,0.2898},
        {0.5711,2.362,5.0349,0.2974},
        {0.5951,2.4123,5.0504,0.3047},
        {0.6198,2.4628,5.0331,0.3118},
        {0.6449,2.5132,5.0528,0.3186},
        {0.6705,2.5637,5.0515,0.3251},
        {0.6966,2.6142,5.051,0.3313},
        {0.7233,2.6648,5.0544,0.3372},
        {0.7505,2.7155,5.0578,0.3428},
        {0.7781,2.7661,5.0731,0.3479},
        {0.8063,2.8167,5.0669,0.3527},
        {0.835,2.8674,5.0675,0.3571},
        {0.8642,2.9182,5.0709,0.361},
        {0.8938,2.969,5.0864,0.3645},
        {0.924,3.0197,5.0791,0.3676},
        {0.9547,3.0706,5.0807,0.3701},
        {0.986,3.1216,5.0842,0.3722},
        {1.0177,3.1725,5.0999,0.3737},
        {1.05,3.2235,5.0915,0.3747},
        {1.0826,3.2744,5.1066,0.3751},
        {1.1159,3.3253,5.0974,0.3749},
        {1.1497,3.3764,5.1005,0.3742},
        {1.184,3.4276,5.1039,0.3728},
        {1.2188,3.4788,5.1203,0.3707},
        {1.254,3.5299,5.1229,0.368},
        {1.2898,3.581,5.1126,0.3646},
        {1.3262,3.6322,5.1169,0.3605},
        {1.363,3.6835,5.1338,0.3557},
        {1.4004,3.7348,5.1218,0.3502},
        {1.4382,3.7861,5.1407,0.3439},
        {1.4766,3.8374,5.1278,0.3368},
        {1.5154,3.8888,5.1476,0.3289},
        {1.5548,3.9402,5.1338,0.3203},
        {1.5949,3.9917,5.1401,0.3107},
        {1.6352,4.0433,5.1728,0.3003},
        {1.6762,4.0947,5.1245,0.2891},
        {1.7176,4.1464,5.184,0.2769},
        {1.7597,4.1978,5.1298,0.2639},
        {1.8021,4.2496,5.192,0.2498},
        {1.8451,4.3011,5.1505,0.2349},
        {1.8887,4.3528,5.1635,0.219},
        {1.9327,4.4047,5.1829,0.2021},
        {1.9772,4.4564,5.1809,0.1841},
        {2.0224,4.5081,5.1679,0.1652},
        {2.0679,4.5601,5.1939,0.1452},
        {2.1141,4.6119,5.1744,0.1241},
        {2.1608,4.6639,5.2015,0.1019},
        {2.2079,4.7158,5.1982,0.0786},
        {2.2556,4.7677,5.1843,0.0542},
        {2.3038,4.8198,5.213,0.0286},
        {2.3525,4.8719,5.209,0.0019},
        {2.4017,4.9238,5.1946,-0.0261},
        {2.4515,4.9761,5.225,-0.0553},
        {2.5016,5.0282,5.2202,-0.0856},
        {2.5525,5.0802,5.2052,-0.1173},
        {2.6039,5.1326,5.2185,-0.1503},
        {2.6557,5.1851,5.2612,-0.1846},
        {2.7081,5.2371,5.2064,-0.2202},
        {2.7609,5.2896,5.2506,-0.2571},
        {2.8144,5.3419,5.2244,-0.2955},
        {2.8683,5.3945,5.2597,-0.3353},
        {2.9228,5.447,5.253,-0.3764},
        {2.9778,5.4994,5.2367,-0.4191},
        {3.0333,5.5522,5.274,-0.4632},
        {3.0893,5.6048,5.2667,-0.5088},
        {3.1459,5.6573,5.2498,-0.556},
        {3.203,5.7102,5.2893,-0.6048},
        {3.2608,5.7629,5.2591,-0.6551},
        {3.3188,5.816,5.3226,-0.7071},
        {3.3774,5.8685,5.2546,-0.7606},
        {3.4367,5.9214,5.2881,-0.8159},
        {3.4965,5.9746,5.317,-0.873},
        {3.5567,6.0277,5.3079,-0.9317},
        {3.6176,6.0806,5.2896,-0.9922},
        {3.6789,6.134,5.3353,-1.0546},
        {3.7407,6.1872,5.3256,-1.1187},
        {3.8032,6.2403,5.3068,-1.1848},
        {3.8661,6.2938,5.355,-1.2527},
        {3.9296,6.347,5.3192,-1.3227},
        {3.9936,6.4007,5.3691,-1.3946},
        {4.058,6.4542,5.3584,-1.4685},
        {4.1231,6.5076,5.339,-1.5445},
        {4.1888,6.5613,5.3648,-1.6227},
        {4.255,6.6153,5.3997,-1.7031},
        {4.3216,6.6692,5.3884,-1.7855},
        {4.3889,6.7229,5.3684,-1.8703},
        {4.4566,6.7771,5.4251,-1.9573},
        {4.5249,6.8309,5.3846,-2.0466},
        {4.5939,6.8852,5.4143,-2.1385},
        {4.6633,6.9397,5.453,-2.2328},
        {4.7331,6.994,5.4409,-2.3295},
        {4.8036,7.0482,5.4203,-2.4287},
        {4.8747,7.1028,5.4529,-2.5306},
        {4.9462,7.1577,5.4945,-2.6352},
        {5.0184,7.2122,5.4504,-2.7424},
        {5.0911,7.2671,5.4854,-2.8526},
        {5.1643,7.3224,5.5294,-2.9655},
        {5.2381,7.3772,5.4838,-3.0814},
        {5.3123,7.4327,5.5548,-3.2002},
        {5.3873,7.4875,5.4745,-3.3221},
        {5.4627,7.5436,5.6084,-3.4473},
        {5.5387,7.599,5.5344,-3.5756},
        {5.6151,7.655,5.6111,-3.7071},
        {5.6923,7.7103,5.5268,-3.842},
        {5.7699,7.767,5.6709,-3.9805},
        {5.8483,7.8226,5.5563,-4.1226},
        {5.927,7.8796,5.7057,-4.2684},
        {6.0063,7.9359,5.6257,-4.4178},
        {6.0863,7.9926,5.6738,-4.571},
        {6.1667,8.0495,5.6925,-4.7283},
        {6.2478,8.1066,5.7119,-4.8896},
        {6.3296,8.1636,5.692,-5.0552},
        {6.4117,8.2219,5.8269,-5.2251},
        {6.4944,8.2792,5.741,-5.3992},
        {6.5779,8.3368,5.7555,-5.578},
        {6.6619,8.3954,5.8562,-5.7616},
        {6.7464,8.4539,5.8458,-5.95},
        {6.8314,8.5125,5.8714,-6.1432},
        {6.9172,8.5711,5.8535,-6.3416},
        {7.0035,8.6307,5.9639,-6.5453},
        {7.0905,8.6899,5.9092,-6.7545},
        {7.178,8.7501,6.0248,-6.9694},
        {7.266,8.8102,6.0173,-7.1898},
        {7.3547,8.8702,6.0018,-7.4162},
        {7.444,8.931,6.0772,-7.649},
        {7.5339,8.9926,6.1634,-7.888},
        {7.6244,9.0537,6.1077,-8.1336},
        {7.7156,9.1156,6.1904,-8.3861},
        {7.8074,9.178,6.2316,-8.6458},
        {7.8997,9.2412,6.3284,-8.9125},
        {7.9928,9.3034,6.2186,-9.1869},
        {8.0866,9.3676,6.4146,-9.4693},
        {8.1808,9.4323,6.4731,-9.7595},
        {8.2758,9.4959,6.3611,-10.0581},
        {8.3714,9.5616,6.573,-10.3655},
        {8.4676,9.6124,5.0711,-10.6817},
        {8.5633,9.5873,-2.5086,-11.0053},
        {8.6589,9.5465,-4.0743,-11.3349},
        {8.754,9.5081,-3.8431,-11.6708},
        {8.8487,9.4682,-3.9906,-12.0126},
        {8.9429,9.4296,-3.8612,-12.3606},
        {9.0368,9.39,-3.9583,-12.7147},
        {9.1303,9.3512,-3.8821,-13.0753},
        {9.2235,9.3125,-3.8642,-13.4427},
        {9.3162,9.2747,-3.785,-13.8165},
        {9.4085,9.2358,-3.8914,-14.1967},
        {9.5005,9.1977,-3.8096,-14.584},
        {9.5921,9.1598,-3.7913,-14.9783},
        {9.6834,9.122,-3.7729,-15.3798},
        {9.7742,9.0852,-3.6865,-15.7883},
        {9.8646,9.0471,-3.8065,-16.2036},
        {9.9547,9.01,-3.718,-16.6265},
        {10.0445,8.973,-3.7,-17.0569},
        {10.1339,8.9361,-3.6822,-17.4951},
        {10.223,8.8994,-3.6646,-17.9411},
        {10.3115,8.8638,-3.5703,-18.3945},
        {10.3998,8.8267,-3.7097,-18.8554},
        {10.4877,8.7905,-3.6143,-19.3245},
        {10.5752,8.7545,-3.5986,-19.8017},
        {10.6625,8.7187,-3.5835,-20.2873},
        {10.7492,8.6839,-3.4837,-20.7807},
        {10.8357,8.6475,-3.6428,-21.2821},
        {10.9218,8.612,-3.543,-21.7921},
        {11.0076,8.5767,-3.5315,-22.3108},
        {11.0931,8.5415,-3.5211,-22.8382},
        {11.1781,8.5073,-3.4178,-23.3739},
        {11.2628,8.4713,-3.5999,-23.918},
        {11.3472,8.4363,-3.4979,-24.471},
        {11.4312,8.4024,-3.3935,-25.0325},
        {11.5149,8.3665,-3.5916,-25.6024},
        {11.5981,8.3326,-3.3857,-26.1809},
        {11.6812,8.2956,-3.7001,-26.7686},
        {11.7638,8.2639,-3.1717,-27.3649},
        {11.846,8.2267,-3.7168,-27.9691},
        {11.928,8.1916,-3.5068,-28.5826},
        {12.0096,8.1576,-3.4042,-29.2048},
        {12.0907,8.1223,-3.5298,-29.8348},
        {12.1716,8.0857,-3.6628,-30.4734},
        {12.2521,8.0512,-3.4462,-31.1206},
        {12.3323,8.0154,-3.5865,-31.7755},
        {12.4122,7.978,-3.7344,-32.4388},
        {12.4915,7.9441,-3.39,-33.1098},
        {12.5706,7.9048,-3.9248,-33.7883},
        {12.6492,7.8704,-3.4486,-34.4742},
        {12.7276,7.8303,-4.0061,-35.1673},
        {12.8055,7.7951,-3.5213,-35.8676},
        {12.883,7.7554,-3.967,-36.5741},
        {12.9602,7.7166,-3.8812,-37.2874},
        {13.037,7.6773,-3.9337,-38.0072},
        {13.1134,7.6388,-3.849,-38.7327},
        {13.1894,7.5969,-4.191,-39.4636},
        {13.2649,7.5572,-3.9671,-40.1996},
        {13.3401,7.514,-4.3228,-40.9404},
        {13.4148,7.473,-4.0982,-41.6859},
        {13.489,7.4298,-4.3176,-42.4348},
        {13.563,7.3844,-4.5425,-43.1877},
        {13.6365,7.3412,-4.316,-43.9444},
        {13.7093,7.2973,-4.3935,-44.7027},
        {13.7819,7.2478,-4.9407,-45.4632},
        {13.8538,7.2039,-4.3963,-46.2255},
        {13.9255,7.1527,-5.1131,-46.9893},
        {13.9965,7.1071,-4.5607,-47.7543},
        {14.0671,7.0558,-5.13,-48.5191},
        {14.1371,7.0069,-4.8919,-49.2833},
        {14.2067,6.9539,-5.3063,-50.0466},
        {14.2756,6.9033,-5.063,-50.8087},
        {14.3441,6.8484,-5.4815,-51.569},
        {14.412,6.7962,-5.2327,-52.3274},
        {14.4795,6.7396,-5.653,-53.0833},
        {14.5464,6.6855,-5.3986,-53.8365},
        {14.6127,6.629,-5.6511,-54.5855},
        {14.6784,6.5717,-5.7305,-55.3301},
        {14.7435,6.5137,-5.8077,-56.0698},
        {14.808,6.4549,-5.8824,-56.8044},
        {14.8721,6.3936,-6.1243,-57.5345},
        {14.9354,6.3368,-5.6763,-58.2588},
        {14.9981,6.2742,-6.2669,-58.976},
        {15.0602,6.2127,-6.1527,-59.6868},
        {15.1217,6.1506,-6.2123,-60.3908},
        {15.1826,6.0878,-6.2685,-61.0879},
        {15.2429,6.0246,-6.3212,-61.7778},
        {15.3025,5.9627,-6.1981,-62.4592},
        {15.3614,5.8968,-6.5956,-63.1319},
        {15.4197,5.8322,-6.4582,-63.7967},
        {15.4774,5.7672,-6.4966,-64.4534},
        {15.5345,5.7017,-6.5316,-65.1019},
        {15.5909,5.6379,-6.3919,-65.7409},
        {15.6466,5.5702,-6.7712,-66.3703},
        {15.7017,5.5039,-6.6164,-66.9909},
        {15.756,5.4394,-6.4675,-67.6016},
        {15.8097,5.371,-6.8363,-68.2024},
        {15.8627,5.3043,-6.6724,-68.7941},
        {15.9151,5.2374,-6.685,-69.3766},
        {15.9668,5.1722,-6.5262,-69.9488},
        {16.0179,5.1015,-7.0472,-70.5118},
        {16.0683,5.0379,-6.3628,-71.0653},
        {16.1179,4.9709,-6.7204,-71.6075},
        {16.1669,4.9003,-7.0534,-72.1402},
        {16.2154,4.8348,-6.5328,-72.6644},
        {16.263,4.7695,-6.5401,-73.1782},
        {16.31,4.7008,-6.8761,-73.6814},
        {16.3563,4.6339,-6.6931,-74.1751},
        {16.402,4.5671,-6.6846,-74.6594},
        {16.447,4.5003,-6.6748,-75.1341},
        {16.4913,4.4336,-6.6636,-75.5994},
        {16.535,4.3671,-6.6513,-76.0552},
        {16.5781,4.3007,-6.638,-76.5017},
        {16.6204,4.2345,-6.6238,-76.9387},
        {16.6621,4.1684,-6.6088,-77.3664},
        {16.7031,4.1025,-6.5931,-77.7847},
        {16.7434,4.0368,-6.5767,-78.1939},
        {16.7832,3.9695,-6.7143,-78.5947},
        {16.8223,3.9056,-6.374,-78.9874},
        {16.8606,3.8422,-6.3692,-79.37},
        {16.8983,3.7739,-6.8281,-79.7435},
        {16.9356,3.709,-6.4725,-80.1099},
        {16.9719,3.6476,-6.1507,-80.4674},
        {17.0077,3.5799,-6.7675,-80.816},
        {17.0429,3.5174,-6.2668,-81.1567},
        {17.0774,3.4516,-6.5615,-81.4896},
        {17.1113,3.3893,-6.2308,-81.8147},
        {17.1447,3.3239,-6.5218,-82.1321},
        {17.1773,3.2619,-6.195,-82.4418},
        {17.2092,3.1987,-6.3389,-82.743},
        {17.2406,3.134,-6.4651,-83.0367},
        {17.2712,3.0727,-6.1414,-83.323},
        {17.3013,3.0084,-6.4277,-83.6018},
        {17.3309,2.9458,-6.2488,-83.8741},
        {17.3597,2.885,-6.0908,-84.1392},
        {17.3879,2.8211,-6.3727,-84.397},
        {17.4154,2.7608,-6.0579,-84.6477},
        {17.4425,2.6957,-6.4727,-84.8921},
        {17.4689,2.6369,-5.8716,-85.1303},
        {17.4946,2.5755,-6.1685,-85.3607},
        {17.5196,2.5128,-6.29,-85.584},
        {17.5443,2.4498,-6.248,-85.8023},
        {17.5682,2.3916,-5.8155,-86.0145},
        {17.5914,2.3309,-6.11,-86.2191},
        {17.6142,2.2671,-6.3572,-86.4177},
        {17.6362,2.2096,-5.7754,-86.6106},
        {17.6578,2.1459,-6.325,-86.7976},
        {17.6786,2.0885,-5.753,-86.9789},
        {17.6989,2.0268,-6.1733,-87.1537},
        {17.7185,1.9667,-6.0083,-87.3227},
        {17.7376,1.9067,-5.9964,-87.4862},
        {17.7561,1.8468,-5.9849,-87.644},
        {17.774,1.787,-5.9738,-87.7963},
        {17.7913,1.7273,-5.9631,-87.9431},
        {17.808,1.6677,-5.9529,-88.0844},
        {17.8241,1.6082,-5.9431,-88.2203},
        {17.8396,1.5489,-5.9338,-88.3508},
        {17.8545,1.4896,-5.9249,-88.4759},
        {17.8687,1.4305,-5.9164,-88.5956},
        {17.8824,1.3716,-5.9085,-88.71},
        {17.8955,1.3128,-5.9011,-88.8192},
        {17.9082,1.2522,-5.9962,-88.9239},
        {17.92,1.1964,-5.632,-89.0234},
        {17.9315,1.1346,-6.127,-89.1177},
        {17.9421,1.0792,-5.6231,-89.2068},
        {17.9523,1.0179,-6.114,-89.2907},
        {17.9619,0.9607,-5.7158,-89.3703},
        {17.9711,0.9002,-5.9399,-89.4456},
        {17.9795,0.8444,-5.6187,-89.5158},
        {17.9872,0.7852,-6.0022,-89.5801},
        {17.9945,0.7255,-5.9214,-89.6402},
        {18.0012,0.6683,-5.6964,-89.696},
        {18.0073,0.61,-5.84,-89.7468},
        {18.0128,0.5521,-5.8421,-89.7925},
        {18.0178,0.492,-5.8749,-89.8341},
        {18.0223,0.434,-5.6852,-89.8715},
        {18.0261,0.3749,-5.8271,-89.9038},
        {18.0292,0.3164,-5.8348,-89.9312},
        {18.0318,0.2587,-5.8534,-89.9536},
        {18.0339,0.1972,-5.7267,-89.9718},
        {18.0352,0.143,-5.9081,-89.9851},
        {18.0361,0.0824,-5.7776,-89.9934}
    };

    public static double[][] pathRight = new double[][] {
        {0,0,0,0.0001},
        {0.0005,0.0414,3.7833,0.0003},
        {0.0012,0.0795,3.985,0.0007},
        {0.0024,0.1276,5.064,0.0015},
        {0.0042,0.1776,4.875,0.0026},
        {0.0065,0.2289,5.1504,0.0042},
        {0.0093,0.2789,5.1032,0.006},
        {0.0126,0.3293,4.9701,0.0083},
        {0.0164,0.3804,5.1133,0.011},
        {0.0207,0.4308,5.0895,0.0139},
        {0.0255,0.4813,4.9961,0.0173},
        {0.0309,0.5323,5.0971,0.021},
        {0.0367,0.5828,5.0795,0.0251},
        {0.0431,0.6334,5.0065,0.0295},
        {0.0498,0.6839,5.1513,0.0342},
        {0.0571,0.7338,4.9943,0.0391},
        {0.065,0.7846,5.0201,0.0445},
        {0.0733,0.8353,5.1318,0.0501},
        {0.0821,0.8853,4.9977,0.056},
        {0.0916,0.9362,5.02,0.0623},
        {0.1014,0.987,5.1154,0.0688},
        {0.1119,1.0372,4.9975,0.0755},
        {0.1226,1.0876,5.104,0.0825},
        {0.1341,1.1377,4.9535,0.0897},
        {0.1459,1.1886,5.1041,0.0971},
        {0.1582,1.2385,5.0315,0.1047},
        {0.1712,1.2887,4.9994,0.1125},
        {0.1846,1.3392,5.0437,0.1204},
        {0.1985,1.3895,5.0318,0.1286},
        {0.2128,1.4397,5.0283,0.1368},
        {0.2278,1.4899,4.992,0.1452},
        {0.2432,1.5403,5.0631,0.1537},
        {0.2591,1.5903,4.979,0.1623},
        {0.2754,1.6405,5.0539,0.171},
        {0.2923,1.6903,4.974,0.1797},
        {0.3098,1.7405,5.0158,0.1884},
        {0.3277,1.7906,5.0051,0.1972},
        {0.3461,1.8406,5.0015,0.206},
        {0.365,1.8905,4.9979,0.2148},
        {0.3844,1.9404,4.9944,0.2235},
        {0.4042,1.9903,4.9908,0.2322},
        {0.4246,2.0401,4.9872,0.2408},
        {0.4455,2.0899,4.9836,0.2493},
        {0.4669,2.1397,4.9801,0.2577},
        {0.4888,2.1895,4.9765,0.2659},
        {0.5112,2.2392,4.9729,0.2741},
        {0.5341,2.2889,4.9693,0.282},
        {0.5575,2.3387,4.9657,0.2898},
        {0.5814,2.3884,4.9621,0.2974},
        {0.6057,2.438,4.9787,0.3047},
        {0.6307,2.4874,4.9306,0.3118},
        {0.656,2.5371,4.9746,0.3186},
        {0.6818,2.5865,4.9433,0.3251},
        {0.7082,2.6359,4.9437,0.3313},
        {0.7351,2.6853,4.9402,0.3372},
        {0.7625,2.7348,4.9366,0.3428},
        {0.7903,2.7842,4.9498,0.3479},
        {0.8186,2.8334,4.9264,0.3527},
        {0.8474,2.8827,4.9257,0.3571},
        {0.8768,2.932,4.9222,0.361},
        {0.9065,2.9812,4.9335,0.3645},
        {0.9368,3.0303,4.913,0.3676},
        {0.9676,3.0795,4.9114,0.3701},
        {0.999,3.1287,4.908,0.3722},
        {1.0308,3.1778,4.9176,0.3737},
        {1.0631,3.2269,4.8999,0.3747},
        {1.0957,3.2758,4.9097,0.3751},
        {1.129,3.3247,4.8933,0.3749},
        {1.1627,3.3737,4.8903,0.3742},
        {1.1971,3.4227,4.887,0.3728},
        {1.2318,3.4717,4.8942,0.3707},
        {1.2669,3.5204,4.8909,0.368},
        {1.3026,3.5692,4.8773,0.3646},
        {1.3388,3.618,4.8731,0.3605},
        {1.3754,3.6667,4.8787,0.3557},
        {1.4126,3.7154,4.8679,0.3502},
        {1.4502,3.7641,4.8711,0.3439},
        {1.4883,3.8127,4.8616,0.3368},
        {1.5269,3.8613,4.8635,0.3289},
        {1.566,3.9099,4.8554,0.3203},
        {1.6057,3.9585,4.8493,0.3107},
        {1.6457,4.0069,4.8585,0.3003},
        {1.6863,4.0555,4.8437,0.2891},
        {1.7272,4.1038,4.8462,0.2769},
        {1.7689,4.1524,4.8387,0.2639},
        {1.8108,4.2006,4.8372,0.2498},
        {1.8533,4.249,4.8381,0.2349},
        {1.8963,4.2973,4.8253,0.219},
        {1.9398,4.3456,4.8254,0.2021},
        {1.9837,4.3938,4.8271,0.1841},
        {2.0281,4.442,4.8206,0.1652},
        {2.073,4.4902,4.8138,0.1452},
        {2.1185,4.5384,4.8142,0.1241},
        {2.1643,4.5864,4.8059,0.1019},
        {2.2106,4.6345,4.8089,0.0786},
        {2.2575,4.6826,4.8042,0.0542},
        {2.3048,4.7305,4.7938,0.0286},
        {2.3525,4.7784,4.7976,0.0019},
        {2.4008,4.8264,4.794,-0.0261},
        {2.4495,4.8742,4.7814,-0.0553},
        {2.4987,4.9219,4.786,-0.0856},
        {2.5484,4.9698,4.7834,-0.1173},
        {2.5987,5.0176,4.7704,-0.1503},
        {2.6492,5.0651,4.7621,-0.1846},
        {2.7004,5.113,4.7825,-0.2202},
        {2.7519,5.1605,4.7553,-0.2571},
        {2.8041,5.2082,4.7647,-0.2955},
        {2.8566,5.2557,4.7461,-0.3353},
        {2.9096,5.3031,4.7527,-0.3764},
        {2.9632,5.3507,4.7526,-0.4191},
        {3.0172,5.398,4.7317,-0.4632},
        {3.0716,5.4454,4.7389,-0.5088},
        {3.1265,5.4928,4.7398,-0.556},
        {3.1819,5.54,4.7164,-0.6048},
        {3.2379,5.5874,4.7308,-0.6551},
        {3.2941,5.6342,4.6988,-0.7071},
        {3.3509,5.6816,4.7354,-0.7606},
        {3.4083,5.7287,4.7022,-0.8159},
        {3.466,5.7756,4.6888,-0.873},
        {3.5242,5.8225,4.6978,-0.9317},
        {3.583,5.8696,4.701,-0.9922},
        {3.6421,5.9163,4.6706,-1.0546},
        {3.7017,5.963,4.6803,-1.1187},
        {3.7618,6.0099,4.6843,-1.1848},
        {3.8223,6.0563,4.651,-1.2527},
        {3.8834,6.1031,4.6723,-1.3227},
        {3.9449,6.1495,4.6371,-1.3946},
        {4.0068,6.1959,4.6478,-1.4685},
        {4.0692,6.2424,4.6531,-1.5445},
        {4.1322,6.2888,4.6276,-1.6227},
        {4.1956,6.3349,4.6069,-1.7031},
        {4.2593,6.381,4.6182,-1.7855},
        {4.3236,6.4273,4.6244,-1.8703},
        {4.3883,6.473,4.5818,-1.9573},
        {4.4535,6.5191,4.6087,-2.0466},
        {4.5192,6.565,4.5794,-2.1385},
        {4.5854,6.6105,4.5544,-2.2328},
        {4.6518,6.6562,4.5666,-2.3295},
        {4.7188,6.7019,4.574,-2.4287},
        {4.7864,6.7473,4.5418,-2.5306},
        {4.8542,6.7924,4.5135,-2.6352},
        {4.9226,6.8379,4.5447,-2.7424},
        {4.9915,6.883,4.5101,-2.8526},
        {5.0608,6.9278,4.4792,-2.9655},
        {5.1305,6.9729,4.5121,-3.0814},
        {5.2006,7.0174,4.4542,-3.2002},
        {5.2713,7.0626,4.5095,-3.3221},
        {5.3423,7.1066,4.4011,-3.4473},
        {5.4139,7.1512,4.4629,-3.5756},
        {5.4857,7.1951,4.3988,-3.7071},
        {5.5582,7.2398,4.4588,-3.842},
        {5.6309,7.2831,4.3395,-3.9805},
        {5.7043,7.3275,4.4302,-4.1226},
        {5.778,7.3705,4.3053,-4.2684},
        {5.8521,7.4142,4.3736,-4.4178},
        {5.9267,7.4575,4.3258,-4.571},
        {6.0017,7.5006,4.3075,-4.7283},
        {6.0771,7.5434,4.2885,-4.8896},
        {6.1531,7.5865,4.2972,-5.0552},
        {6.2293,7.6283,4.1858,-5.2251},
        {6.306,7.6709,4.2605,-5.3992},
        {6.3832,7.7133,4.2351,-5.578},
        {6.4607,7.7547,4.1462,-5.7616},
        {6.5387,7.7963,4.157,-5.95},
        {6.617,7.8376,4.1317,-6.1432},
        {6.6958,7.879,4.1391,-6.3416},
        {6.775,7.9194,4.0403,-6.5453},
        {6.8547,7.9603,4.0846,-6.7545},
        {6.9347,8.0001,3.9803,-6.9694},
        {7.015,8.0399,3.9882,-7.1898},
        {7.0958,8.0798,3.9935,-7.4162},
        {7.177,8.1191,3.9187,-7.649},
        {7.2585,8.1575,3.8436,-7.888},
        {7.3405,8.1963,3.8893,-8.1336},
        {7.4228,8.2344,3.8073,-8.3861},
        {7.5056,8.2721,3.7667,-8.6458},
        {7.5886,8.3089,3.6806,-8.9125},
        {7.6721,8.3466,3.7708,-9.1869},
        {7.756,8.3825,3.5856,-9.4693},
        {7.8401,8.4178,3.5375,-9.7595},
        {7.9247,8.4542,3.6304,-10.0581},
        {8.0095,8.4884,3.429,-10.3655},
        {8.0947,8.5094,2.0921,-10.6817},
        {8.1792,8.4564,-5.3091,-11.0053},
        {8.2632,8.3972,-5.9126,-11.3349},
        {8.3466,8.3355,-6.1633,-11.6708},
        {8.4294,8.2755,-6.0063,-12.0126},
        {8.5114,8.214,-6.1456,-12.3606},
        {8.5929,8.1537,-6.039,-12.7147},
        {8.6739,8.0925,-6.1154,-13.0753},
        {8.7543,8.0311,-6.1334,-13.4427},
        {8.8339,7.969,-6.2226,-13.8165},
        {8.9129,7.9079,-6.1064,-14.1967},
        {8.9914,7.846,-6.1883,-14.584},
        {9.0693,7.7839,-6.2067,-14.9783},
        {9.1466,7.7216,-6.2252,-15.3798},
        {9.2231,7.6585,-6.3217,-15.7883},
        {9.299,7.5966,-6.1918,-16.2036},
        {9.3743,7.5338,-6.2803,-16.6265},
        {9.4491,7.4708,-6.2983,-17.0569},
        {9.5232,7.4076,-6.3162,-17.4951},
        {9.5967,7.3442,-6.3337,-17.9411},
        {9.6694,7.2798,-6.4382,-18.3945},
        {9.7416,7.217,-6.2886,-18.8554},
        {9.8131,7.1532,-6.384,-19.3245},
        {9.884,7.0891,-6.3996,-19.8017},
        {9.9543,7.0249,-6.4146,-20.2873},
        {10.0238,6.9598,-6.5245,-20.7807},
        {10.0928,6.8962,-6.3551,-21.2821},
        {10.1611,6.8317,-6.4547,-21.7921},
        {10.2288,6.767,-6.4661,-22.3108},
        {10.2959,6.7022,-6.4763,-22.8382},
        {10.3622,6.6363,-6.5898,-23.3739},
        {10.4279,6.5724,-6.3971,-23.918},
        {10.493,6.5073,-6.4988,-24.471},
        {10.5574,6.4412,-6.6134,-25.0325},
        {10.6212,6.3772,-6.4046,-25.6024},
        {10.6842,6.311,-6.6206,-26.1809},
        {10.7468,6.2481,-6.2849,-26.7686},
        {10.8085,6.1797,-6.8445,-27.3649},
        {10.8697,6.117,-6.2779,-27.9691},
        {10.9303,6.052,-6.4874,-28.5826},
        {10.9901,5.986,-6.6002,-29.2048},
        {11.0493,5.9213,-6.4742,-29.8348},
        {11.1079,5.858,-6.33,-30.4734},
        {11.1658,5.7924,-6.5568,-31.1206},
        {11.2231,5.7283,-6.416,-31.7755},
        {11.2798,5.6656,-6.2567,-32.4388},
        {11.3358,5.5995,-6.6222,-33.1098},
        {11.3912,5.5388,-6.065,-33.7883},
        {11.4459,5.4732,-6.5624,-34.4742},
        {11.5001,5.4134,-5.9823,-35.1673},
        {11.5535,5.3485,-6.4884,-35.8676},
        {11.6064,5.2883,-6.031,-36.5741},
        {11.6586,5.2271,-6.116,-37.2874},
        {11.7103,5.1664,-6.0628,-38.0072},
        {11.7614,5.1049,-6.1579,-38.7327},
        {11.8118,5.0468,-5.8039,-39.4636},
        {11.8617,4.9865,-6.0383,-40.1996},
        {11.911,4.9297,-5.6703,-40.9404},
        {11.9597,4.8707,-5.9056,-41.6859},
        {12.0078,4.8139,-5.6854,-42.4348},
        {12.0555,4.7593,-5.448,-43.1877},
        {12.1025,4.7024,-5.6852,-43.9444},
        {12.1489,4.6463,-5.6188,-44.7027},
        {12.1949,4.5958,-5.0469,-45.4632},
        {12.2403,4.5397,-5.6144,-46.2255},
        {12.2852,4.4909,-4.8726,-46.9893},
        {12.3296,4.4364,-5.4484,-47.7543},
        {12.3735,4.3878,-4.866,-48.5191},
        {12.4168,4.3366,-5.1157,-49.2833},
        {12.4597,4.2898,-4.688,-50.0466},
        {12.5021,4.2404,-4.9431,-50.8087},
        {12.544,4.1953,-4.5111,-51.569},
        {12.5855,4.1476,-4.7719,-52.3274},
        {12.6266,4.1041,-4.3378,-53.0833},
        {12.6672,4.0581,-4.6045,-53.8365},
        {12.7073,4.0146,-4.3514,-54.5855},
        {12.747,3.9719,-4.2713,-55.3301},
        {12.7863,3.9299,-4.1935,-56.0698},
        {12.8252,3.8888,-4.1183,-56.8044},
        {12.8638,3.8501,-3.8618,-57.5345},
        {12.9018,3.8068,-4.3371,-58.2588},
        {12.9395,3.7695,-3.7322,-58.976},
        {12.9768,3.731,-3.8459,-59.6868},
        {13.0137,3.6932,-3.7858,-60.3908},
        {13.0503,3.6558,-3.7292,-61.0879},
        {13.0865,3.619,-3.676,-61.7778},
        {13.1223,3.581,-3.8139,-62.4592},
        {13.1577,3.547,-3.4011,-63.1319},
        {13.1928,3.5116,-3.5382,-63.7967},
        {13.2276,3.4766,-3.4994,-64.4534},
        {13.2621,3.4419,-3.4641,-65.1019},
        {13.2961,3.4057,-3.6199,-65.7409},
        {13.3299,3.3734,-3.2245,-66.3703},
        {13.3633,3.3396,-3.3791,-66.9909},
        {13.3963,3.3042,-3.5449,-67.6016},
        {13.429,3.2727,-3.1594,-68.2024},
        {13.4614,3.2394,-3.3232,-68.7941},
        {13.4935,3.2063,-3.3105,-69.3766},
        {13.5252,3.1715,-3.4874,-69.9488},
        {13.5567,3.1421,-2.9306,-70.5118},
        {13.5877,3.1056,-3.6514,-71.0653},
        {13.6183,3.0727,-3.2949,-71.6075},
        {13.6488,3.0434,-2.9244,-72.1402},
        {13.679,3.0087,-3.4638,-72.6644},
        {13.7087,2.974,-3.4766,-73.1782},
        {13.7381,2.9428,-3.1217,-73.6814},
        {13.7672,2.9098,-3.3049,-74.1751},
        {13.7959,2.8766,-3.3138,-74.6594},
        {13.8244,2.8434,-3.3241,-75.1341},
        {13.8525,2.81,-3.3357,-75.5994},
        {13.8803,2.7765,-3.3485,-76.0552},
        {13.9077,2.7429,-3.3624,-76.5017},
        {13.9348,2.7091,-3.3772,-76.9387},
        {13.9615,2.6752,-3.3929,-77.3664},
        {13.9879,2.6411,-3.4093,-77.7847},
        {14.014,2.6069,-3.4265,-78.1939},
        {14.0398,2.5742,-3.2652,-78.5947},
        {14.0652,2.5378,-3.63,-78.9874},
        {14.0901,2.5013,-3.6614,-79.37},
        {14.1148,2.4698,-3.1529,-79.7435},
        {14.1392,2.4346,-3.5081,-80.1099},
        {14.1631,2.3959,-3.8837,-80.4674},
        {14.1868,2.3637,-3.2144,-80.816},
        {14.21,2.3263,-3.743,-81.1567},
        {14.233,2.2921,-3.421,-81.4896},
        {14.2555,2.2543,-3.7806,-81.8147},
        {14.2778,2.2195,-3.4613,-82.1321},
        {14.2996,2.1813,-3.8184,-82.4418},
        {14.321,2.1446,-3.6762,-82.743},
        {14.3421,2.1094,-3.5199,-83.0367},
        {14.3628,2.0708,-3.8762,-83.323},
        {14.3831,2.0352,-3.5583,-83.6018},
        {14.4031,1.9977,-3.7372,-83.8741},
        {14.4227,1.9585,-3.9301,-84.1392},
        {14.4419,1.9223,-3.6147,-84.397},
        {14.4607,1.8827,-3.9663,-84.6477},
        {14.4793,1.8477,-3.4789,-84.8921},
        {14.4974,1.8062,-4.1543,-85.1303},
        {14.515,1.7677,-3.8605,-85.3607},
        {14.5322,1.7308,-3.7013,-85.584},
        {14.5493,1.6935,-3.702,-85.8023},
        {14.5658,1.6513,-4.2168,-86.0145},
        {14.5818,1.6123,-3.9262,-86.2191},
        {14.5977,1.5763,-3.592,-86.4177},
        {14.6129,1.5338,-4.2638,-86.6106},
        {14.628,1.4973,-3.6224,-86.7976},
        {14.6425,1.4545,-4.2895,-86.9789},
        {14.6567,1.4163,-3.8228,-87.1537},
        {14.6704,1.3764,-3.9884,-87.3227},
        {14.6838,1.3364,-4.0009,-87.4862},
        {14.6968,1.2963,-4.0131,-87.644},
        {14.7094,1.256,-4.0249,-87.7963},
        {14.7215,1.2156,-4.0363,-87.9431},
        {14.7333,1.175,-4.0474,-88.0844},
        {14.7446,1.1344,-4.0582,-88.2203},
        {14.7556,1.0937,-4.0687,-88.3508},
        {14.7661,1.0529,-4.0788,-88.4759},
        {14.7762,1.0121,-4.0886,-88.5956},
        {14.7859,0.9712,-4.0982,-88.71},
        {14.7952,0.9303,-4.1075,-88.8192},
        {14.8042,0.8906,-3.9305,-88.9239},
        {14.8126,0.8463,-4.4626,-89.0234},
        {14.8207,0.808,-3.7955,-89.1177},
        {14.8283,0.7638,-4.4868,-89.2068},
        {14.8355,0.7257,-3.8043,-89.2907},
        {14.8423,0.6827,-4.2996,-89.3703},
        {14.8489,0.6423,-3.9618,-89.4456},
        {14.8548,0.5975,-4.5181,-89.5158},
        {14.8603,0.5577,-4.0283,-89.5801},
        {14.8656,0.5177,-3.968,-89.6402},
        {14.8703,0.4742,-4.3262,-89.696},
        {14.8746,0.4324,-4.1906,-89.7468},
        {14.8785,0.3908,-4.2002,-89.7925},
        {14.8821,0.3503,-3.9611,-89.8341},
        {14.8852,0.3061,-4.3337,-89.8715},
        {14.8879,0.2635,-4.2029,-89.9038},
        {14.8901,0.2212,-4.2156,-89.9312},
        {14.8919,0.1794,-4.2389,-89.9536},
        {14.8933,0.1379,-3.8621,-89.9718},
        {14.8942,0.0925,-4.9492,-89.9851},
        {14.8948,0.0549,-3.5932,-89.9934}
    };
    // public static TrajectoryPoint[] trajectoryPointsLeft = new TrajectoryPoint[pathLeft.length];
    // public static TrajectoryPoint[] trajectoryPointsRight = new TrajectoryPoint[pathRight.length];
    // public static void createPath(double[][] path, TrajectoryPoint[] trajectoryPoint) {
    //     restart = true;
    //     for (int i = 0; i < path.length; i++) {
    //         double position;
    //         double velocity;
    //         double acceleration;
    //         double heading;
    //         TrajectoryPoint point = new TrajectoryPoint();
    //         point.position = path[i][0];
    //         point.velocity = path[i][1];
    //         point.headingDeg = path[i][3];
    //         trajectoryPoint[i] = (point);
    //     }
    // }
}