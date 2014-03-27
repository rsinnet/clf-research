#!/usr/bin/perl
#

use strict;
use warnings;

use File::Spec;
use File::Basename;

if ($#ARGV != 1) {
    print "Usage: (LINUX)    ./math2mat.pl srcpath targetpath\n";
    print "       (WINDOWS)  perl.exe math2mat.pl srcpath targetpath\n";
    exit;
}

my $srcpath = $ARGV[0];
my $targetpath = $ARGV[1];

my @filenames;

# use the glob operator to find the Mathematica output files:
push @filenames, $_ for glob(File::Spec->catfile($srcpath,"*.math"));

# Parse the mathematica output files and write them to disk in a format usable by MATLAB:
for (my $matnum = 0; $matnum < scalar(@filenames); $matnum++) {
    my $filename = fileparse($filenames[$matnum], '\.[^\.]*');
    my $srcfile = File::Spec->catfile($srcpath, $filename.".math");
    my $targetfile = File::Spec->catfile($targetpath, $filename.".m");
    open(DAT, "< $srcfile") || die("Could not open file: $srcfile");
    my @raw_data = <DAT>;
    close DAT;
    
    my $mat_str = "\n";
    
    $mat_str .= "x$filename = ";
     
    foreach my $linevar (@raw_data) {
        $linevar =~ s/\},/\};/g;
        $linevar =~ s/\n{1,2}/ ...\n/g;
        $linevar =~ s/Pi/pi/g;

	$linevar =~ s/Abs/abs/g;
	$linevar =~ s/Sign/sign/g;

	$linevar =~ s/ArcCos/acos/g;
	$linevar =~ s/Sqrt/sqrt/g;

        $linevar =~ s/Sec/sec/g;
        $linevar =~ s/Csc/csc/g;
        $linevar =~ s/Cot/cot/g;
        
        $linevar =~ s/Cos/cos/g;
        $linevar =~ s/Sin/sin/g;
        $linevar =~ s/Tan/tan/g;

	$linevar =~ s/E/exp(1)/g;
        
        $linevar =~ s/\[/\(/g;
        $linevar =~ s/\]/\)/g;
        $linevar =~ s/\{/\[/g;
        $linevar =~ s/\}/\]/g;
        
	# can't figure out why the character appears: 
        $linevar =~ s///g;

        $mat_str .= $linevar;
    }
    $mat_str .= ";\n";

    my @function_arguments;
    
    # Figure out which variables we need to include based on which are present in the expressions
    # the following conditions on the if block perform a regex search for variables in
    # the output.
    my @args = ('q', 'dq');
    #foreach (@args) {
	# check for vectors
	#if ($mat_str =~ m/\([^\(\)]*$_\)/ ||
	    # and for multidimensional arrays
	    #$mat_str =~ m/$_\(\s*\d{1,4}\s*(,\s*\d{1,4}\s*)*\)/) {
	    # add the current argument to the list
	    #push(@function_arguments, $_);
	#}
    #}

    # Tack p onto the end, regardless of whether or not it is present for the sake of simplifying
    # function calls in MATLAB.
    push(@function_arguments, 'q');
    push(@function_arguments, 'dq');
    push(@function_arguments, 'p');
    
    # set the default list separator behavior. See: perldoc perlvar
    $" = ", ";
    my $mat_header = "function x$filename = $filename(@function_arguments)\n";

    # concatenate the header and the equation
    $mat_str = $mat_header.$mat_str;


    # write the equation to disk.
    open(DAT, "> $targetfile") || die "Can't open $targetfile : $!";
    print DAT $mat_str;
    close DAT;

}
