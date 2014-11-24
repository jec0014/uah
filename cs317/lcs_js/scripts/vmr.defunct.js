<!-- Defunct code, Refactor or Remove

<!--
			<DIV class="img_wrapper">
				<IMG class="class_img" src="../images/database/person/psn_0001.jpg" />
			</DIV>
			<DIV class="img_wrapper">
				<IMG class="class_img" src="../images/database/person/psn_0002.jpg" />
			</DIV>
			<DIV class="img_wrapper">
				<IMG class="class_img" src="../images/database/person/psn_0003.jpg" />
			</DIV>
			<BR>
			<DIV class="img_wrapper">
				<IMG class="class_img" src="../images/database/person/psn_0004.jpg" />
			</DIV>
			<DIV class="img_wrapper">
				<IMG class="class_img" src="../images/database/person/psn_0005.jpg" />
			</DIV>
			<DIV class="img_wrapper">
				<IMG class="class_img" src="../images/database/person/psn_0006.jpg" />
			</DIV>
			<BR>
			<DIV class="img_wrapper">
				<IMG class="class_img" src="../images/database/person/psn_0007.jpg" />
			</DIV>
			<DIV class="img_wrapper">
				<IMG class="class_img" src="../images/database/person/psn_0008.jpg" />
			</DIV>
			<DIV class="img_wrapper">
				<IMG class="class_img" src="../images/database/person/psn_0009.jpg" />
			</DIV>
			-->

/*
		$(document).find(".class_img").dblclick(function(e) {
			vmr.panzoom.panzoom('zoom', 25.0, {
				animate: true,
				focal: e
			});
		});
		*/

		
		/*
		$(document).find(".img_overlay").click(function(e) {
			$(this).parent().find(".img_wrapper").addClass("selected");
		});
		*/

/*
.img_wrapper:after {
	content: '\A';
	position: absolute;
	width: 100%;
	height: 100%;
	top: 0;
	left: 0;
	background: rgba(0,100,0,0.6);
	opacity: 0;
	trasition: all 0.5s;
	-webkit-transition: all 0.5s;
}

.img_wrapper:hover:after {
	opacity: 1;
}
*/

	<SCRIPT src="../scripts/masonry.pkgd.min.js"></SCRIPT>
	<SCRIPT src="../scripts/freewall.js"></SCRIPT>

	/*
	var searchDiv = $('#bin_left_imgs');
	searchDiv.empty();
	$('.selected_bin_left').clone().each( function (index) {
		searchDiv.append("<DIV class=\"img_wrapper\" id=\"bin_left_img_" + index + "\"></DIV>");
		searchDiv.find("#bin_left_img_" + index).append(this);
	});
	
	var searchDiv = $('#bin_bottom_imgs');
	searchDiv.empty();
	$('.selected_bottom_top').clone().each( function (index) {
		searchDiv.append("<DIV class=\"img_wrapper\" id=\"bin_bottom_img_" + index + "\"></DIV>");
		searchDiv.find("#bin_bottom_img_" + index).append(this);
	});
	
	var searchDiv = $('#bin_right_imgs');
	searchDiv.empty();
	$('.selected_right_top').clone().each( function (index) {
		searchDiv.append("<DIV class=\"img_wrapper\" id=\"bin_right_img_" + index + "\"></DIV>");
		searchDiv.find("#bin_right_img_" + index).append(this);
	});
	* */

/*
			switch (c2) {
				
			case 1:
				switchElem.find('#wrapper_overview_' + c2).css({
					"width": wrapperWidth[c1][c2],
					"height": wrapperHeight[c1][c2],
					//"background": "url(../images/ratio_database/" + db_img[c1][0] + "/" + db_img[c1][c2] + ") no-repeat center",
					//"background-size": wrapperWidth[c1][c2] + "px " +  wrapperHeight[c1][c2] + "px"
				});
			break;
			
			case 2:
				switchElem.find('#wrapper_overview_' + c2).css({
					"width": wrapperWidth[c1][c2],
					"height": wrapperHeight[c1][c2],
					//"background": "url(../images/ratio_database/" + db_img[c1][0] + "/" + db_img[c1][c2] + ") no-repeat center",
					//"background-size": wrapperWidth[c1][c2] + "px " +  wrapperHeight[c1][c2] + "px"
				});
				switchElem.append('<BR>');
			break;
			
			case 3:
				switchElem.find('#wrapper_overview_' + c2).css({
					"width": wrapperWidth[c1][c2],
					"height": wrapperHeight[c1][c2],
					//"background": "url(../images/ratio_database/" + db_img[c1][0] + "/" + db_img[c1][c2] + ") no-repeat center",
					//"background-size": wrapperWidth[c1][c2] + "px " +  wrapperHeight[c1][c2] + "px"
				});
			break;
			
			case 4:
				switchElem.find('#wrapper_overview_' + c2).css({
					"width": wrapperWidth[c1][c2],
					"height": wrapperHeight[c1][c2],
					//"background": "url(../images/ratio_database/" + db_img[c1][0] + "/" + db_img[c1][c2] + ") no-repeat center",
					//"background-size": wrapperWidth[c1][c2] + "px " +  wrapperHeight[c1][c2] + "px"
				});
			break;
			
			case 5:
				switchElem.find('#wrapper_overview_' + c2).css({
					"width": wrapperWidth[c1][c2],
					"height": wrapperHeight[c1][c2],
					//"background": "url(../images/ratio_database/" + db_img[c1][0] + "/" + db_img[c1][c2] + ") no-repeat center",
					//"background-size": wrapperWidth[c1][c2] + "px " +  wrapperHeight[c1][c2] + "px"
				});
			break;
			
			}
			*/

function toggleOverview (element) {
			element.addClass('invisible');
			if (element.hasClass('subPane')) {
				
			}
		}

/*
		var wall = new freewall('#subPane_imgFrame_' + db_img[c1][0]);
		wall.reset({
			selector: '.item',
			animate: false,
			onResize: function() {
				wall.fitWidth();
			}
		});
		wall.fitZone((subPaneWidth[c1]-1), (subPaneHeight[c1]-23));
		*/

/*
			var localElement = $(this).parent();
			console.log(localElement);
			console.log(localElement.hasClass('overview_toggle_0'));
			$(localElement).addClass('invisible');
			var newElem = $(localElement).clone();
			$(newElem).replaceWith($('.subPane.overview_toggle_0'));
			$(newElem).removeClass('invisible');
			*/
			//if (newElem.hasClass('overview_toggle_0')) {
			//	$(newElem).toggleClass('invisible');
			//}

	$element.panzoom().on('panzoomzoom', function(e, panzoom, scale, opts) {
			vmr.currentScale = scale;
			vmr.increment = (vmr.currentScale / 2);
			//console.log("Scale: ", scale);
			//console.log("Border: ", (2 / scale));
			$section.find('.class_img').css("border-width", (2 / scale));
			//$section.find('.img_wrapper').css("margin-top", (4 / scale));
			$section.find('.img_wrapper').css("margin", (2 / scale));
			
			$section.find('.class_subPane').css("border-width", (2 / scale));
			//$section.find('.class_subPane').css("padding-right", (4 / scale));
			
			$section.find('.subPaneFooter').css("border-width", (2 / scale));
			fontSize = ((10 / scale) + (6));
			$section.find('.subPaneFooter').css("font-size", fontSize + "px");
		});

startTransform: 'scale(1.0)',
				increment: 0.1,
				minScale: 0.5,
				contain: false
				
-->
