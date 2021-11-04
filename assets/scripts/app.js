function makeWordCloud(words) {
  $(".teaching-domains").jQCloud(words, { delay: 120 });
}

function displayWordCloud() {
  var count = 1;
  $(window).on("scroll", function () {
    var y_scroll_pos = window.pageYOffset;
    var scroll_pos_test = 2700; // set to whatever you want it to be
    var words = makeWords();
    if (y_scroll_pos > scroll_pos_test && count <= 1) {
      makeWordCloud(words);
      count++;
    }
  });
}

function designForm() {
  $("#my-modal form").addClass("my-form");
}

$(document).ready(function () {
  Typed.new("#writing-text", {
    strings: [
      "un développeur Full-Stack junior. ",
      '"webophage"',
      "prêt à relever des défis.",
    ],
    // Optionally use an HTML element to grab strings from (must wrap each string in a <p>)
    stringsElement: null,
    // typing speed
    typeSpeed: 1,
    contentType: "text",
    callback: function () {
      $("#writing-text").css({ color: "#fff", "background-color": "#C8412B" });
    },
    preStringTyped: function () {},
    onStringTyped: function () {},
  });

  displayWordCloud();
});
