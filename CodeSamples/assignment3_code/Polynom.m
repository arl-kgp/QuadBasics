function [Poly]=Polynom(n,derivative,t)
%returns a vector with the coefficients of the derivatives of a n grade
%polynomial. n= number of polonomials, t= time we want to compute it at.

Poly=ones(n,1);

if derivative~=0
    for i=1:derivative
        Poly=polyder(Poly);    
    end
    for i=1:derivative
        Poly(end+1)=0;    
    end
    
    Poly=fliplr(Poly);
end

grade=zeros(derivative+1,1);
grade(derivative+1:n)=linspace(0,n-(derivative+1),n-derivative);
grade=grade';
Poly=Poly.*(t.^grade);

end