function [out ,outp,len] = Format_to_type(in)
outp=1;
len=1;
out='uint8';
switch in
    case 'b'
        out='int8';
        len=1;
    case 'B'
        out='uint8';
        len=1;
    case 'h'
        out='int16';
        len=2;
    case 'H'
        out='uint16';
        len=2;
    case 'i'
        out='int32';
        len=4;
    case 'I'
        out='uint32';
        len=4;
    case 'f'
        out='single';
        len=4;
    case 'd'
        out='double';
        len=8;
    case 'c'
        out='int16';
        outp=100;
        len=1;
    case 'C'
        out='uint16';
        outp=100;
        len=2;
    case 'e'
        out='int32';
        outp=100;
        len=4;
    case 'E'
        out='uint32';
        outp=100;
        len=4;
    case 'q'
        out='int64';
        len=8;
    case 'Q'
        out='uint64';
        len=8;
    otherwise
        out='int8';
end

end

% b   : int8
% B   : uint8
% h   : int16
% H   : uint16
% i   : int32
% I   : uint32
% f   : float
% d   : double
% c   : int16  * 100
% C   : uint16* 100
% e   : int32  * 100
% E   : uint32 * 100
% q   : int64
% Q   : uint64