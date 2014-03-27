(* ::Package:: *)

BeginPackage["pbar`",{"JLink`"}];

ShowProgressBar[title_String:"Computation Progress", caption_String:"Percent complete:", percent_Integer:0] :=
	Module[{frame, panel, label, bar},
		InstallJava[];
		bar = JavaNew["javax.swing.JProgressBar"];
		JavaBlock[
			frame = JavaNew["javax.swing.JFrame", title];
			frame@setSize[300, 110];
			frame@setResizable[False];
			frame@setLocation[400, 400];
			panel = JavaNew["javax.swing.JPanel"];
			panel@setLayout[Null];
			frame@getContentPane[]@add[panel];
			label = JavaNew["javax.swing.JLabel", caption];
			label@setBounds[20, 10, 260, 20];
			panel@add[label];
			bar@setBounds[20, 40, 260, 30];
			bar@setMinimum[0];
			bar@setMaximum[100];
			bar@setValue[percent];
			panel@add[bar];
			JavaShow[frame];
			bar
		]
	]

DestroyProgressBar[bar_?JavaObjectQ] :=
	JavaBlock[
		LoadClass["javax.swing.SwingUtilities"];
		SwingUtilities`windowForComponent[bar]@dispose[];
		ReleaseObject[bar]
	]

PSimple[mtx_] := Module[{m,n,i,j,rmtx,bar},
    bar = ShowProgressBar[];
	m = Dimensions[mtx][[1]];
	n = Dimensions[mtx][[2]];
    rmtx = 0 * mtx;
    For[i=1,i<=m,i++,
      For[j=1,j<=n,j++,
		rmtx[[i,j]] = Simplify[mtx[[i,j]]];
		bar@setValue[Ceiling[100((i - 1) * n + j) / (m*n)]];
		bar@update;
	  ]
	];
	DestroyProgressBar[bar];
	rmtx
];

EndPackage[];
