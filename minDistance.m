function dist = minDistance(x, path)
   dist = inf;
   for i = 1:size(path) - 1
      currentDist = minDistanceSegment(x, path(i,:), path(i+1,:));
      if currentDist < dist
          dist = currentDist;
      end
   end
end
        
function dist = minDistanceSegment(e, pA, pB)
    AB = pB - pA;
    BE = e - pB;
    AE = e - pA;
    AB_BE = dot(AB, BE);
    AB_AE = dot(AB, AE);

    if AB_BE > 0
        dist = norm(BE);
    elseif AB_AE < 0
        dist = norm(AE);
    else
        dist = abs(AB(1) * AE(2) - AB(2) * AE(1)) / norm(AB);
    end
end