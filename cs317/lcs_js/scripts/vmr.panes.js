function generatePanes () {

var subPaneWidth = [];
var subPaneHeight = [];

var img_num_across = [[10,7,24,21,20],[10,7,10,14,13],[13,12,27,27,26],[13,12,27,27,26],[10,9,17,20,16],[5,5,5,5,5]];
var device = 4; // 0-smap, 1-1024x768, 2-1920x1080, 3-macbook, 4-kharselim, 5-testing(small)
var docWidth = $(document).width();

var urlDevice = getQueryVariable("device");

if (urlDevice) {
	// Usage: file:///home/berocs/html_vmr/html/main.html?device=0
	console.log("Setting 'device' from URL.");
	device = urlDevice;
} else if (docWidth === 1920) { // 1920x1080
	device = 2;
} else if (docWidth === 1024) { //1024x768
	device = 1;
} else if (docWidth === 1366) { // 1366x768
	device = 4;
}

console.log("Device: ", device);

var img_num_tall = [1,1,1,1,1]; // Generated later

vmr.num_subPane = db_img.length;

// Generate detail subPanes
for (var c1 = 0; c1 < vmr.num_subPane; c1++)
{
	// Create subPane
	vmr.panzoom.append("<DIV class=\"class_subPane overview_toggle_" + c1 + "\" id=\"subPane_" + db_img[c1][0] + "\"></DIV>");
	
	// Store subPane in local variable to speed up generation
	vmr.detail_subPane[c1] = $("#subPane_" + db_img[c1][0]);
	
	//vmr.detail_subPane[c1].hide();
	
	vmr.num_imgs += db_img[c1].length;
	
	var c2_height_limit = 10;
	
	switch (device) {
		case '0':
		case 0: // smap
			c2_height_limit = 10;
		break;
		case '1':
		case 1: // 1024x768
			c2_height_limit = 8;
		break;
		case '2':
		case 2: // 1920x1080
			c2_height_limit = 12;
		break;
		case '5':
		case 5: // testing
			c2_height_limit = 5;
		break;
	}
	
	//console.log("len: ", db_img[c1].length);
	
	var c2 = 1;
	
	while ((img_num_tall[c1] <= c2_height_limit))
	{
		c2_index = (c2%(db_img[c1].length-1))+1;
		
		// Insert image wrapper (has border)
		vmr.detail_subPane[c1].append("<DIV class=\"img_wrapper\" id=\"wrapper" + c2 + "\"></DIV>");
		
		var detail_wrapper = vmr.detail_subPane[c1].find("#wrapper" + c2);
		
		// Insert image
		detail_wrapper.append("<IMG class=\"class_img\" src=\"../images/database/" + db_img[c1][0] + "/" + db_img[c1][c2_index] + "\" />");
		
		// Generate bin buttons for each image
		detail_wrapper.append("<IMG class=\"img_overlay img_overlay_top\" src=\"../images/overlay_top.png\" />");
		
		// Generate bin buttons for each image
		detail_wrapper.append("<IMG class=\"img_overlay img_overlay_left\" src=\"../images/overlay_left.png\" />");
		
		// Generate bin buttons for each image
		detail_wrapper.append("<IMG class=\"img_overlay img_overlay_bottom\" src=\"../images/overlay_bottom.png\" />");
		
		// Generate bin buttons for each image
		detail_wrapper.append("<IMG class=\"img_overlay img_overlay_right\" src=\"../images/overlay_right.png\" />");
		
		// [0][] - SMAP 1600x900 - 41 imgs
		// [1][] - 1024x768 - 27 imgs
		// [2][] - 1920x1080 - 53 imgs
		switch (c1) {
			case 0: // Buildings
				if (c2%(img_num_across[device][c1]) === 0) {
					vmr.detail_subPane[c1].append("<BR>");
					img_num_tall[c1]++;
				}
			break;
			case 1: // Misc
				if (c2%(img_num_across[device][c1]) === 0) {
					vmr.detail_subPane[c1].append("<BR>");
					img_num_tall[c1]++;
				}
			break;
			case 2: // People
				if (c2%(img_num_across[device][c1]) === 0) {
					vmr.detail_subPane[c1].append("<BR>");
					img_num_tall[c1]++;
				}
			break;
			case 3: // Vehicles
				if (c2%(img_num_across[device][c1]) === 0) {
					vmr.detail_subPane[c1].append("<BR>");
					img_num_tall[c1]++;
				}
			break;
			case 4: // Weapons
				if (c2%(img_num_across[device][c1]) === 0) {
					vmr.detail_subPane[c1].append("<BR>");
					img_num_tall[c1]++;
				}
			break;
			default: // Default action
				console.log("error in image <br> generation switch");
			break;
		} /* end switch */
		
		c2++;
		
	} /* end while c2 */
	
	// Attach subPane footer
	vmr.detail_subPane[c1].append("<DIV class=\"subPaneFooter\"></DIV>");
	
	var detail_subPaneFooter = vmr.detail_subPane[c1].find(".subPaneFooter");
	
	// Insert subPane background cluster icon
	detail_subPaneFooter.css("background-image", "url(../images/cluster_icon/cluster_detail_" + c1 + ".svg)");
	
	// Insert image count
	detail_subPaneFooter.append('<div class="text">' + (c2-1) + '</div>');
	
	subPaneWidth[c1] = vmr.detail_subPane[c1].width();
	subPaneHeight[c1] = vmr.detail_subPane[c1].height();
	
} /* end for c1 */


var imgFrameWidth = [];
var imgFrameHeight = [];

// Generate Overview subPanes
for (var c1 = 0; c1 < vmr.num_subPane; c1++)
{
	// Create subPane
	vmr.detail_subPane[c1].prepend("<DIV class=\"subPane overview_toggle_" + c1 + " overview\" id=\"subPane_overview_" 
		+ db_img[c1][0] + "\"></DIV>");
	
	vmr.overview_subPane[c1] = $('#subPane_overview_' + db_img[c1][0]);
	
	//overview_subPane.hide();
	
	vmr.overview_subPane[c1].width(subPaneWidth[c1] + 'px');
	vmr.overview_subPane[c1].height(subPaneHeight[c1] + 'px');
	
	vmr.overview_subPane[c1].append('<DIV id="subPane_imgFrame_' + db_img[c1][0] + '" class="imgFrame overview" style="height: ' 
		+ (subPaneHeight[c1]-28) + 'px; width: ' + (subPaneWidth[c1]) 
		+ 'px; text-align: center; overflow: hidden; display: inline-block;"></DIV>');
	
	var overview_img_num = 6;
	var overview_img_top_num = 2;
	var br_flag = false;
	
	imgFrameWidth[c1] = vmr.overview_subPane[c1].width();
	imgFrameHeight[c1] = vmr.overview_subPane[c1].height();
	
	var wrapperWidth = [[],[],[],[],[]];
	var wrapperHeight = [[],[],[],[],[]];
	var wrapperPaddingTop = [1,1,1,1,1];
	
	if (c1 === 2 && device === 1) {
		overview_img_num = 10;
		overview_img_top_num = 4;
	}
	
	for (var c2 = 1; c2 < db_img[c1].length && c2 < overview_img_num; c2++)
	{
		
		/*
		// Insert image wrapper (has border)
		$('body').find('#subPane_imgFrame_' + db_img[c1][0])
			.append("<DIV class=\"img_wrapper overview item\" id=\"wrapper_overview_" + c2 + "\"></DIV>");
		
		// Insert image
		$('body').find('#subPane_imgFrame_' + db_img[c1][0]).find("#wrapper_overview_" + c2)
			.append("<IMG class=\"class_img overview\" src=\"../images/database/"
			+ db_img[c1][0] + "/" + db_img[c1][c2] + "\" />");
		*/
		
		// Insert image wrapper (has border)
		$('#subPane_imgFrame_' + db_img[c1][0])
			.append("<IMG class=\"img_wrapper overview item\" id=\"wrapper_overview_" + c2 	+ '"' 
			+ ' src="../images/ratio_database/' + db_img[c1][0] + "/" + db_img[c1][c2] + '"'
			+ ' ></IMG>');
			//+ ' style="width: ' + Math.floor((subPaneWidth[c1]-1)/(c2+2)) + 'px; height: ' + Math.floor((subPaneHeight[c1]-23)/(c2+2)) 
			//+ "px; background: url(../images/database/" + db_img[c1][0] + "/" + db_img[c1][c2] + ') no-repeat center;"'
			//+ " ></DIV>");
		
		wrapperWidth[c1][c2] = (Math.floor((imgFrameWidth[c1])/2)-9);
		wrapperHeight[c1][c2] = (Math.floor((imgFrameHeight[c1])/2));
		
		if (c2 > overview_img_top_num) {
			wrapperWidth[c1][c2] = (Math.floor((imgFrameWidth[c1])/3)-12);
			wrapperHeight[c1][c2] = (Math.floor((imgFrameHeight[c1])/3));
		}
		
		if ( (wrapperWidth[c1][c2] / wrapperHeight[c1][c2]) > (16/9) ) {
			//console.log("old: " + wrapperWidth[c1][c2]);
			wrapperWidth[c1][c2] = Math.floor(wrapperHeight[c1][c2] * (16/9));
			//console.log("new: " + wrapperWidth[c1][c2]);
		} else if ( (wrapperWidth[c1][c2] / wrapperHeight[c1][c2]) < 1 ) {
			wrapperPaddingTop[c1] = wrapperPaddingTop[c1] + Math.floor((wrapperHeight[c1][c2] - wrapperWidth[c1][c2]) / 3);
			wrapperHeight[c1][c2] = Math.floor(wrapperWidth[c1][c2]);
		}
		
		wrapperWidth[c1][c2] = wrapperWidth[c1][c2] + 'px';
		wrapperHeight[c1][c2] = wrapperHeight[c1][c2] + 'px';
		
		//console.log(wrapperWidth[c1][c2] + " , " + wrapperHeight[c1][c2]);
		
		var switchElem = $('#subPane_imgFrame_' + db_img[c1][0]);
		
		if (c2 < overview_img_top_num) {
			switchElem.find('#wrapper_overview_' + c2).css({
				"width": wrapperWidth[c1][c2],
				"height": wrapperHeight[c1][c2]
			});
		} else {
			if (br_flag === false) {
				switchElem.append('<BR>');
				br_flag = true;
			}
			switchElem.find('#wrapper_overview_' + c2).css({
				"width": wrapperWidth[c1][c2],
				"height": wrapperHeight[c1][c2]
			});
		}
		
		/*
		$('body').find("." + db_img[c1][0]).find("#wrapper" + c2)
		.append("<IMG class=\"img_overlay\" src=\"../images/img_overlay.png\" />");
		*/
	} /* end for c2 */
	
	$('#subPane_imgFrame_' + db_img[c1][0]).css("height", ((subPaneHeight[c1] - 23 - wrapperPaddingTop[c1]) + 'px'));
	
	vmr.overview_subPane[c1].css({
		"padding-top": (wrapperPaddingTop[c1] + 'px'),
		"height": ((subPaneHeight[c1] - wrapperPaddingTop[c1]) + 'px')
		});
	
	
	// Attach subPane footer
	vmr.overview_subPane[c1].append("<DIV class=\"subPaneFooter overview\"></DIV>");
	
	// Insert subPane background cluster icon
	vmr.overview_subPane[c1].find(".subPaneFooter.overview")
		.css("background-image", "url(../images/cluster_icon/cluster_detail_" + c1 + ".svg)");
	
	// Insert image count
	vmr.overview_subPane[c1].find(".subPaneFooter.overview")
		.append('<div class="text overview">' + c2 + '</div>');
	
} /* end for c1 */


} /* end function generatePanes */
