from scipy.stats import norm

range = 0.7

z = norm.ppf(0.9995)  
sigma = range / z       

prob = norm.cdf(range, 0, sigma) - norm.cdf(-range, 0, sigma)
print(sigma)
print(prob)