abf = trackingABF('MotionModel', '1D Constant Acceleration', 'Coefficients', [1,0.01,0.001], 'State', [1,2,1].');
p = abf.State;
cfs = abf.Coefficients;
cfs(3) = cfs(3)/2;
fprintf('\n\nconst auto x0 = xva<double> { %f, %f, %f };\n', abf.State);
fprintf('auto p = ghk<double> { %f, %f, %f/2 };\n', abf.Coefficients)
fprintf('auto err = 1e-5;\n');
fprintf('auto check_res = [&err](result<double> ep, xva<double> e, xva<double> p){\n');
fprintf('  EXPECT_NEAR(ep.update.x, e.x, err);\n')
fprintf('  EXPECT_NEAR(ep.update.v, e.v, err);\n')
fprintf('  EXPECT_NEAR(ep.update.a, e.a, err);\n')
fprintf('  EXPECT_NEAR(ep.prediction.x, p.x, err);\n')
fprintf('  EXPECT_NEAR(ep.prediction.v, p.v, err);\n')
fprintf('  EXPECT_NEAR(ep.prediction.a, p.a, err);\n')
fprintf('};\n')

N = 4;
T = 0.1;
z = 1:N;
fprintf('auto res = result<double> { x0, x0 };\n')
for i = 1:N
    X = correct(abf, z(i));
Y = predict(abf, T);

[u,p] = ghk(cfs, z(i), T, p);
assert(all(abs(X-u)<1e-4))
assert(all(abs(Y-p)<1e-4))
fprintf('res = update_predict(p, res.prediction, %f, %f);\n', z(i), T);
fprintf('check_res(res, {%f,%f,%f}, {%f,%f,%f});\n', X, Y);
end