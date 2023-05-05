import numpy as np
from scipy.special import comb
import pdb

class Bezier():

	def __init__(self):
		self.a = 1;

	def bezier(self,coeff,s):
		
		[n,m] = coeff.shape
		y = s.size

		m=m-1 #Bezier polynomials have m terms for m-1 order

		fcn = np.zeros((n,y))
		for k in range(m+1):
			fcn = fcn + np.expand_dims(coeff[:,k],axis=1)*np.expand_dims(self.singleterm_bezier(m,k,s), axis=0)

		return fcn
    
	def singleterm_bezier(self,m,k,s):
  
		if (k == 0):
		    val = comb(m,k)*(1-s)**(m-k)
		elif (m == k):
		    val = comb(m,k)*s**(k)
		else:
		    val = comb(m,k)*s**(k)*(1-s)**(m-k)

		return val


	def diff_coeff(self, coeff):

		M = coeff.shape[1]-1;
		A = np.zeros((M,M+1));

		for i in range(M):
			A[i,i] = -(M-i)*comb(M,i)/comb(M-1,i);
			A[i,i+1] = (i+1)*comb(M,i+1)/comb(M-1,i);
		

		A[M-1,M]=M*comb(M,M)
		dcoeff = coeff@A.T
		return dcoeff


	def dbezier(self, coeff, s):
		dcoeff = self.diff_coeff(coeff);
		fcn = self.bezier(dcoeff,s);
		return fcn