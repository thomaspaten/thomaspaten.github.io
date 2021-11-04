import Question from "./Question.js";
import Quiz from "./Quiz.js";

const App = (() => {
  // cache the DOM
  const quizEl = document.querySelector(".quiz");
  const quizQuestionEl = document.querySelector(".quiz__question");
  const trackerEl = document.querySelector(".quiz__tracker");
  const taglineEl = document.querySelector(".quiz__tagline");
  const choicesEl = document.querySelector(".quiz__choices");
  const progressInnerEl = document.querySelector(".progress__inner");
  const nextButtonEl = document.querySelector(".next");
  const restartButtonEl = document.querySelector(".restart");

  const q1 = new Question(
    "________ est un outil de définition et d'exécution d'applications Docker multi-conteneurs.",
    ["Docker Swarn", "Docker Hub", "Docker Cloud", "Docker Compose"],
    3
  );
  const q2 = new Question(
    "Quelle est la commande pour obtenir l'état actuel du repository Git ?",
    ["git getStatus", "--status", "git status", "git config --status"],
    2
  );
  const q3 = new Question(
    "En ReactJS, les props sont __________ dans d'autres composants",
    [
      "des méthodes",
      "injecter",
      "injecter et des méthodes",
      "Toute ses réponses",
    ],
    0
  );
  const q4 = new Question(
    "ReactJS utilise _____ pour augmenter les performances.",
    [
      "Original Dom",
      "Virtual DOM",
      " Original DOM et Virtual DOM",
      "Aucune de ses réponses",
    ],
    1
  );
  const q5 = new Question(
    "Combien de colonnes sont extraites de cette requête ? SELECT address1||','||address2||','||address2 'Address' FROM  employee;",
    [1, 2, 3, 4],
    2
  );

  const quiz = new Quiz([q1, q2, q3, q4, q5]);

  const listeners = (_) => {
    nextButtonEl.addEventListener("click", function () {
      const selectedRadioElem = document.querySelector(
        'input[name="choice"]:checked'
      );
      if (selectedRadioElem) {
        const key = Number(selectedRadioElem.getAttribute("data-order"));
        quiz.guess(key);
        renderAll();
      }
    });

    restartButtonEl.addEventListener("click", function () {
      // 1. reset the quiz
      quiz.reset();
      // 2. renderAll
      renderAll();
      // 3. restore the next button
      nextButtonEl.style.opacity = 1;
    });
  };

  const setValue = (elem, value) => {
    elem.innerHTML = value;
  };

  const renderQuestion = (_) => {
    const question = quiz.getCurrentQuestion().question;
    setValue(quizQuestionEl, question);
  };

  const renderChoicesElements = (_) => {
    let markup = "";
    const currentChoices = quiz.getCurrentQuestion().choices;
    currentChoices.forEach((elem, index) => {
      markup += `
        <li class="quiz__choice">
          <input type="radio" name="choice" class="quiz__input" data-order="${index}" id="choice${index}">
          <label for="choice${index}" class="quiz__label">
            <i></i>
            <span>${elem}</span>
          </label>
        </li>
      `;
    });

    setValue(choicesEl, markup);
  };

  const renderTracker = (_) => {
    const index = quiz.currentIndex;
    setValue(trackerEl, `${index + 1} de ${quiz.questions.length}`);
  };

  const getPercentage = (num1, num2) => {
    return Math.round((num1 / num2) * 100);
  };

  const launch = (width, maxPercent) => {
    let loadingBar = setInterval(function () {
      if (width > maxPercent) {
        clearInterval(loadingBar);
      } else {
        width++;
        progressInnerEl.style.width = width + "%";
      }
    }, 3);
  };

  const renderProgress = (_) => {
    // 1. width
    const currentWidth = getPercentage(
      quiz.currentIndex,
      quiz.questions.length
    );
    // 2. launch(0, width)
    launch(0, currentWidth);
  };

  const renderEndScreen = (_) => {
    setValue(quizQuestionEl, `Beau travail !`);
    setValue(taglineEl, `C'est complet !`);
    setValue(
      trackerEl,
      `Votre score est de ${getPercentage(quiz.score, quiz.questions.length)}%`
    );
    nextButtonEl.style.opacity = 0;
    renderProgress();
  };

  const renderAll = (_) => {
    if (quiz.hasEnded()) {
      // renderEndScreen
      renderEndScreen();
    } else {
      // 1. render the question
      renderQuestion();
      // 2. Render the choices elements
      renderChoicesElements();
      // 3. Render Tracker
      renderTracker();
      // 4. Render Progress
      renderProgress();
    }
  };

  return {
    renderAll: renderAll,
    listeners: listeners,
  };
})();

App.renderAll();
App.listeners();
