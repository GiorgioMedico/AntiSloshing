function Y = screwvector(e,p,pitch)

if isempty(pitch)
    h = 0;
else
    h = pitch;
end

Y = [e; cross(p,e) + h*e;];

end