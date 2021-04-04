function r = r_ref(t,flag)
  
if (flag == 1)
    
    r = zeros(length(t),1);
    
    for i = 1:length(t)
        if (t(i)<1)
            r(i,1) = 0;
        elseif(t(i)<10)
            r(i,1) = 5;
        elseif(t(i)<22)
            r(i,1) = 0;
        elseif(t(i)<35)
            r(i,1) = -5;
        elseif(t(i)<45)
            r(i,1) = 0;
        elseif(t(i)<55)
            r(i,1) = 10;
        elseif(t(i)<65)
            r(i,1) = 0;
        elseif(t(i)<75)
            r(i,1) = -10;
        elseif(t(i)<90)
            r(i,1) = 0;
        elseif(t(i)<98)
            r(i,1) = 5;
        elseif(t(i)<110)
            r(i,1) = 0;
        elseif(t(i)<120)
            r(i,1) = -5;
        elseif(t(i)<=140)
            r(i,1) = 0;
        else
            r(i,1) = 0;
        end
    end
        
         r = deg2rad(r);
         
elseif (flag == 2)
    
    r = 0.1745*sin(t);
    
else
    
    r = 0;
    
end
         
         
end