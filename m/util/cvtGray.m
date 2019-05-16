
function G = cvtGray(I, width, height)
	%% Returns a greyscaled representation G of I.
	G = reshape(normalize(I, 0, 255), height, width);
	G = uint8(G);
end
