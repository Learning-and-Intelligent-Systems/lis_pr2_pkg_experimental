%!PS-Adobe-3.0
%%Creator: (ImageMagick)
%%Title: (geometry_msgs.msg)
%%CreationDate: (2017-03-02T20:40:24-05:00)
%%BoundingBox: 1673 1310 1692 1334
%%HiResBoundingBox: 1673 1310 1692 1334
%%DocumentData: Clean7Bit
%%LanguageLevel: 1
%%Orientation: Portrait
%%PageOrder: Ascend
%%Pages: 1
%%EndComments

%%BeginDefaults
%%EndDefaults

%%BeginProlog
%
% Display a color image.  The image is displayed in color on
% Postscript viewers or printers that support color, otherwise
% it is displayed as grayscale.
%
/DirectClassPacket
{
  %
  % Get a DirectClass packet.
  %
  % Parameters:
  %   red.
  %   green.
  %   blue.
  %   length: number of pixels minus one of this color (optional).
  %
  currentfile color_packet readhexstring pop pop
  compression 0 eq
  {
    /number_pixels 3 def
  }
  {
    currentfile byte readhexstring pop 0 get
    /number_pixels exch 1 add 3 mul def
  } ifelse
  0 3 number_pixels 1 sub
  {
    pixels exch color_packet putinterval
  } for
  pixels 0 number_pixels getinterval
} bind def

/DirectClassImage
{
  %
  % Display a DirectClass image.
  %
  systemdict /colorimage known
  {
    columns rows 8
    [
      columns 0 0
      rows neg 0 rows
    ]
    { DirectClassPacket } false 3 colorimage
  }
  {
    %
    % No colorimage operator;  convert to grayscale.
    %
    columns rows 8
    [
      columns 0 0
      rows neg 0 rows
    ]
    { GrayDirectClassPacket } image
  } ifelse
} bind def

/GrayDirectClassPacket
{
  %
  % Get a DirectClass packet;  convert to grayscale.
  %
  % Parameters:
  %   red
  %   green
  %   blue
  %   length: number of pixels minus one of this color (optional).
  %
  currentfile color_packet readhexstring pop pop
  color_packet 0 get 0.299 mul
  color_packet 1 get 0.587 mul add
  color_packet 2 get 0.114 mul add
  cvi
  /gray_packet exch def
  compression 0 eq
  {
    /number_pixels 1 def
  }
  {
    currentfile byte readhexstring pop 0 get
    /number_pixels exch 1 add def
  } ifelse
  0 1 number_pixels 1 sub
  {
    pixels exch gray_packet put
  } for
  pixels 0 number_pixels getinterval
} bind def

/GrayPseudoClassPacket
{
  %
  % Get a PseudoClass packet;  convert to grayscale.
  %
  % Parameters:
  %   index: index into the colormap.
  %   length: number of pixels minus one of this color (optional).
  %
  currentfile byte readhexstring pop 0 get
  /offset exch 3 mul def
  /color_packet colormap offset 3 getinterval def
  color_packet 0 get 0.299 mul
  color_packet 1 get 0.587 mul add
  color_packet 2 get 0.114 mul add
  cvi
  /gray_packet exch def
  compression 0 eq
  {
    /number_pixels 1 def
  }
  {
    currentfile byte readhexstring pop 0 get
    /number_pixels exch 1 add def
  } ifelse
  0 1 number_pixels 1 sub
  {
    pixels exch gray_packet put
  } for
  pixels 0 number_pixels getinterval
} bind def

/PseudoClassPacket
{
  %
  % Get a PseudoClass packet.
  %
  % Parameters:
  %   index: index into the colormap.
  %   length: number of pixels minus one of this color (optional).
  %
  currentfile byte readhexstring pop 0 get
  /offset exch 3 mul def
  /color_packet colormap offset 3 getinterval def
  compression 0 eq
  {
    /number_pixels 3 def
  }
  {
    currentfile byte readhexstring pop 0 get
    /number_pixels exch 1 add 3 mul def
  } ifelse
  0 3 number_pixels 1 sub
  {
    pixels exch color_packet putinterval
  } for
  pixels 0 number_pixels getinterval
} bind def

/PseudoClassImage
{
  %
  % Display a PseudoClass image.
  %
  % Parameters:
  %   class: 0-PseudoClass or 1-Grayscale.
  %
  currentfile buffer readline pop
  token pop /class exch def pop
  class 0 gt
  {
    currentfile buffer readline pop
    token pop /depth exch def pop
    /grays columns 8 add depth sub depth mul 8 idiv string def
    columns rows depth
    [
      columns 0 0
      rows neg 0 rows
    ]
    { currentfile grays readhexstring pop } image
  }
  {
    %
    % Parameters:
    %   colors: number of colors in the colormap.
    %   colormap: red, green, blue color packets.
    %
    currentfile buffer readline pop
    token pop /colors exch def pop
    /colors colors 3 mul def
    /colormap colors string def
    currentfile colormap readhexstring pop pop
    systemdict /colorimage known
    {
      columns rows 8
      [
        columns 0 0
        rows neg 0 rows
      ]
      { PseudoClassPacket } false 3 colorimage
    }
    {
      %
      % No colorimage operator;  convert to grayscale.
      %
      columns rows 8
      [
        columns 0 0
        rows neg 0 rows
      ]
      { GrayPseudoClassPacket } image
    } ifelse
  } ifelse
} bind def

/DisplayImage
{
  %
  % Display a DirectClass or PseudoClass image.
  %
  % Parameters:
  %   x & y translation.
  %   x & y scale.
  %   label pointsize.
  %   image label.
  %   image columns & rows.
  %   class: 0-DirectClass or 1-PseudoClass.
  %   compression: 0-none or 1-RunlengthEncoded.
  %   hex color packets.
  %
  gsave
  /buffer 512 string def
  /byte 1 string def
  /color_packet 3 string def
  /pixels 768 string def

  currentfile buffer readline pop
  token pop /x exch def
  token pop /y exch def pop
  x y translate
  currentfile buffer readline pop
  token pop /x exch def
  token pop /y exch def pop
  currentfile buffer readline pop
  token pop /pointsize exch def pop
  /Times-Roman findfont pointsize scalefont setfont
  x y scale
  currentfile buffer readline pop
  token pop /columns exch def
  token pop /rows exch def pop
  currentfile buffer readline pop
  token pop /class exch def pop
  currentfile buffer readline pop
  token pop /compression exch def pop
  class 0 gt { PseudoClassImage } { DirectClassImage } ifelse
  grestore
  showpage
} bind def
%%EndProlog
%%Page:  1 1
%%PageBoundingBox: 1673 1310 1692 1334
DisplayImage
1673 1310
19 24
12
19 24
0
0
C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6
C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C6C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1
C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C1C159574E
59574E59574E59574E59574E59574E59574E59574E59574E59574E59574E59574E59574E59574E
59574E59574E59574E59574E59574E626055626055626055626055626055626055626055626055
62605562605562605562605562605562605562605562605562605562605562605559574F59574F
59574F59574F59574F59574F59574F59574F59574F59574F59574F59574F59574F59574F59574F
59574F59574F59574F59574F58564E58564E58564E58564E58564E58564E58564E58564E58564E
58564E58564E58564E58564E58564E58564E58564E58564E58564E58564E57554D57554D57554D
57554D57554D57554D57554D57554D57554D57554D57554D57554D57554D57554D57554D57554D
57554D57554D57554D56544C56544C56544C56544C56544C56544C56544C56544C56544C56544C
56544C56544C56544C56544C56544C56544C56544C56544C56544C55534B55534B55534B55534B
55534B55534B55534B55534B55534B55534B55534B55534B55534B55534B55534B55534B55534B
55534B55534B54524A54524A54524A54524A54524A54524A54524A54524A54524A54524A54524A
54524A54524A54524A54524A54524A54524A54524A54524A535149535149535149535149535149
535149535149535149535149535149535149535149535149535149535149535149535149535149
5351493D3D3A544F48525049525049525049504F4F3E3A38343333333333333333333434393A38
48433C554F4852504952504952504941454B333333D4A060463C39534E47514F484F4E4E3E3E4F
7FB1C5DCDBD2DFDBD2DFDBD2DFD6C6C7B7A094795B463A39584D42514F4840454C4371AFDFDBD2
DFDAB07A4938594A40504E474E4D4E3E3B518DC5D2DFDBD2DFDBD2DDDAD2DFDBD2DFDBD2DFDBD2
DCBA875D413A584C443E3F45538CBEDFDBD2DFDBD2CB8F53463E37514D464D4C4D3D3B518DC5D2
DFDBD2C5854D373333353C48617FA1D0DBD2DFDBD2CD9457483D373D3B4260A3CADFDBD2D3DBD2
DFD09A633E39574B414C4B4C3D3B518DC5D2DFDBD2C5854D4940374C4B4A3B383F5E9AC5DFDBD2
DFCF975C3B383C3A436EB4D2DFDBD297CBD2DFDBCFB270434D41374B4A4B3C3B518DC5D2DFDBD2
C5854D493F364E4B443F444C3E66A7DCDBD2DFDBB07443373A394B82BFD2DFDBD2538CBEDFDBD2
DFBA774C3A384E494A3C3B518DC5D2DFDBD2C5854D493E364D4A443D434B3E68A8DDDBD2DFDBB0
734337393B5493C9D2DFDBCEDFDBD2DFDBD2DFDBBB894F395245413B3B518DC5D2DFDBD2C5854D
483E354646483A384166A2C8DFDBD2DFCD935A3B37383C5BA2D0D2DFDBC7CDCBC4D8DBD2DFDBD2
CB9053453B363A3B518DC5D2DFDBD2C5854D3733343843516D8DADD8DBD2DFDBD2C78D52453B38
393D62B0D8D2DFDBC23434395894C2DFDBD2DFCC925B3B36393A518DC5D2DFDBD2DFDBD2DFDBD2
DFDBD2DFDBD2DFDBD2D5AF7C553D3749444939426DB9DBD2DFDBBD3C424C3B558CCCDBD2DFDBC7
A05E3D383B4D7DAEC2DBDBD2DFDBD2DFDBD2DFD4C3C3B19A8C71534238364E453D3F444B394979
C1DBD2DFDBB946453F3B424A3533333333344D4135454544383836333333333333333333333434
373736433F3649453F46453F46453F3F434A36333333333545443F45443F45443F45443F45443F
45443F45443F45443F45443F45443F45443F45443F45443F45443F45443F45443F45443F45443F
45443F

%%PageTrailer
%%Trailer
%%EOF
