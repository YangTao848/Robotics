function Diff = dif( linex1,linex2,x1,x2 )
  line=linex1-linex2;
  a=x1-linex2;
  b=x2-linex2;
  prod1=line(1)*a(2)-line(2)*a(1);
  prod2=line(1)*b(2)-line(2)*b(1);
  if(prod1*prod2<0);
      Diff=true;
  else Diff=false;
  end
end

