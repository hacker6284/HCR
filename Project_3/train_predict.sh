svm-train -s 0 -t 2 -c $1 -g $2 rad_d2 rad_d2.predict
svm-predict rad_d2.t rad_d2.predict rad_d2.t.predict
